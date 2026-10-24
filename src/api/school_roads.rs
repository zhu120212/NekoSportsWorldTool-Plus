//! 自动获取学校区域 OSM 路网（Overpass 公共实例）。
//!
//! 只提交公开地图查询，**不发送任何账号凭据**。下载到的 XML 原样落盘成 `.osm`，
//! 与手工导入的文件同格式，因此把路径写进 `config.osm_path` 即可被跑步流程直接使用。
//!
//! 范围来源两选一：服务端下发的电子围栏（BD-09），或当前学校的真实打卡点位。
//! 围栏与点位都是 BD-09，Overpass 的 bbox 必须 WGS-84，故先做反解。

use route_planner::RoadGraph;
use sha2::{Digest, Sha256};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// 单次响应体积上限（24 MiB）。
const MAX_BYTES: u64 = 24 * 1024 * 1024;
/// 缓存有效期：7 天。
const TTL_MS: i64 = 7 * 24 * 60 * 60 * 1000;
/// 主键 + 官方镜像 + 两个社区实例（国内常被拦，挂代理时可用）。
/// 实测（2026-10，国内直连）：前两个可达，后两个连接失败但迅速失败，不会拖慢整体。
const ENDPOINTS: [&str; 4] = [
    "https://overpass-api.de/api/interpreter",
    "https://z.overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
];
/// 单个端点最多尝试次数，以及两次之间的退避秒数。
const ATTEMPTS: u32 = 2;
const RETRY_WAIT_S: u64 = 3;
/// 单个端点的 HTTP 超时。
///
/// 实测（2026-10，北京城区，含建筑）：1 km × 1 km 约 107 秒（811 KB），
/// 其中服务端查询约 30 秒、回传约 70 秒；只查道路时 32 秒。
/// 120 秒会卡在 1 km² 边缘，2 km² 以上必被截断成半个文件（缺 `</osm>`，随后被
/// `parse` 判为不完整）。定 240 秒，覆盖到 4 km² 量级的校园。
const HTTP_TIMEOUT: Duration = Duration::from_secs(240);
/// overpass-api.de 前置的 nginx 对浏览器/curl 风格 UA 一律返回 406，必须用非标准 UA。
const UA: &str = "NekoSportsWorldTool/0.4.2 (campus-road-map)";
/// 查询内部超时：60 会被前置 nginx 判成 504，实测 25 最稳。
const QUERY_TIMEOUT_S: u32 = 25;
/// 下载范围跨度上限（度）：再大 Overpass 会直接拒绝。
const MAX_SPAN_LAT: f64 = 0.15;
const MAX_SPAN_LON: f64 = 0.2;
/// 围栏外扩：把校门外的路口一起纳进来，否则路线会在门口断掉。
const FENCE_MARGIN: f64 = 0.001;
/// 无围栏时按点位取范围的外扩。
const POINT_MARGIN: f64 = 0.002;

/// 围栏合法性：非空、每环至少 3 点、坐标有限、面积非零。
pub fn validate_fences(fences: &[Vec<(f64, f64)>]) -> Result<(), String> {
    if fences.is_empty() {
        return Err("当前学校没有有效围栏，请先刷新电子围栏".into());
    }
    for ring in fences {
        if ring.len() < 3
            || ring.iter().any(|&(lat, lon)| {
                !lat.is_finite() || !lon.is_finite() || lat.abs() > 85.0 || lon.abs() > 180.0
            })
        {
            return Err("学校围栏坐标无效".into());
        }
        let (oy, ox) = ring[0];
        let area: f64 = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .map(|(&(y, x), &(v, u))| (x - ox) * (v - oy) - (u - ox) * (y - oy))
            .sum();
        if area.abs() < 1e-12 {
            return Err("学校围栏面积无效".into());
        }
    }
    Ok(())
}

/// BD-09 → WGS-84：用与正向变换相同的公式迭代反推，避免引入第二套坐标。
///
/// 16 次迭代后残差在 1e-9 度量级，远小于 bbox 本身 220 米的外扩。
pub fn to_wgs(lat: f64, lon: f64) -> (f64, f64) {
    let (mut y, mut x) = (lat, lon);
    for _ in 0..16 {
        let (by, bx) = crate::track::geom::wgs84_to_bd09(y, x);
        y += lat - by;
        x += lon - bx;
    }
    (y, x)
}

/// 由点位算 WGS-84 四至（south, west, north, east），外扩 margin 度。
fn bbox_of(points: &[(f64, f64)], margin: f64) -> Result<(f64, f64, f64, f64), String> {
    if points.is_empty()
        || points
            .iter()
            .any(|&(a, b)| !a.is_finite() || !b.is_finite() || a.abs() > 85. || b.abs() > 180.)
    {
        return Err("缺少有效点位，不能确定路网下载范围".into());
    }
    let (mut south, mut west, mut north, mut east) = (90f64, 180f64, -90f64, -180f64);
    for &(lat, lon) in points {
        let (y, x) = to_wgs(lat, lon);
        south = south.min(y);
        north = north.max(y);
        west = west.min(x);
        east = east.max(x);
    }
    if north - south > MAX_SPAN_LAT || east - west > MAX_SPAN_LON {
        return Err("路网下载范围跨度过大，请确认当前学校后重试".into());
    }
    Ok((south - margin, west - margin, north + margin, east + margin))
}

/// 四至的近似边长（公里），仅用于给用户报进度，不做精确大地线计算。
pub fn span_km((south, west, north, east): (f64, f64, f64, f64)) -> (f64, f64) {
    let lat_km = (north - south) * 111.0;
    let lon_km = (east - west) * 111.0 * ((north + south) * 0.5).to_radians().cos();
    (lat_km.abs(), lon_km.abs())
}

/// 拼 Overpass 查询：道路 + 建筑（建筑用于路线避障）。
pub fn build_query((south, west, north, east): (f64, f64, f64, f64)) -> String {
    let bbox = format!("{south:.7},{west:.7},{north:.7},{east:.7}");
    format!(
        "[out:xml][timeout:{QUERY_TIMEOUT_S}];\
         (way[highway]({bbox});way[building]({bbox});relation[building]({bbox}););\
         (._;>;);out body;"
    )
}

/// 按围栏算范围。
fn query_fences_bbox(fences: &[Vec<(f64, f64)>]) -> Result<(f64, f64, f64, f64), String> {
    validate_fences(fences)?;
    let points: Vec<(f64, f64)> = fences.iter().flatten().copied().collect();
    bbox_of(&points, FENCE_MARGIN)
}

/// 按点位算范围（无围栏时用）。
fn query_points_bbox(points: &[(f64, f64)]) -> Result<(f64, f64, f64, f64), String> {
    bbox_of(points, POINT_MARGIN)
}

/// 按围栏拼查询。
fn query_fences(fences: &[Vec<(f64, f64)>]) -> Result<String, String> {
    Ok(build_query(query_fences_bbox(fences)?))
}

/// 按点位拼查询（无围栏时用）。
fn query_points(points: &[(f64, f64)]) -> Result<String, String> {
    Ok(build_query(query_points_bbox(points)?))
}

/// 提交查询：每个端点最多 `ATTEMPTS` 次，5xx 之间退避，全部失败才报错。
///
/// Overpass 公共实例会随机返回 429（槽位占满）/ 502（过载）/ 504（超时），
/// 同一范围重试一次通常就能过，因此这里不是「换端点」而是「先退避再重试，不行再换」。
pub fn download(query: &str, log: &mut dyn FnMut(&str)) -> Result<Vec<u8>, String> {
    let mut last = String::from("没有可用的 Overpass 端点");
    for endpoint in ENDPOINTS {
        for attempt in 0..ATTEMPTS {
            if attempt > 0 {
                log(&format!("[road] 服务器忙，{RETRY_WAIT_S} 秒后重试"));
                std::thread::sleep(Duration::from_secs(RETRY_WAIT_S));
            }
            log(&format!("[road] 请求 {endpoint}"));
            match once(endpoint, query) {
                Ok(bytes) => return Ok(bytes),
                Err(e) => {
                    log(&format!("[road] 失败：{e}"));
                    // 只有服务器侧的忙/错才值得重试；网络不通换端点才有意义。
                    if !is_retryable(&e) {
                        break;
                    }
                    last = e;
                }
            }
        }
    }
    Err(last)
}

/// 值得重试的：HTTP 429 / 5xx。其他（DNS 失败、连接被拒、超时）换端点也是同样结果。
fn is_retryable(err: &str) -> bool {
    ["429", "500", "502", "503", "504"].iter().any(|c| {
        err.contains(&format!("status code {c}")) || err.starts_with(&format!("{c} "))
    })
}

/// 构造路网下载用的客户端。
///
/// 业务请求一律走 `api::client::make_agent()`，这里单独建一个，只为了加长超时并允许
/// 用户用环境变量挂代理 —— 国内直连 Overpass 常年不通，没有代理这个模块基本废掉一半。
fn build_agent() -> ureq::Agent {
    let mut builder = ureq::AgentBuilder::new().timeout(HTTP_TIMEOUT);
    if let Some(addr) = proxy_from_env() {
        if let Ok(p) = ureq::Proxy::new(&addr) {
            builder = builder.proxy(p);
        }
    }
    builder.build()
}

/// 依次探测 `ALL_PROXY` `all_proxy` `HTTPS_PROXY` `https_proxy`。
fn proxy_from_env() -> Option<String> {
    for key in ["ALL_PROXY", "all_proxy", "HTTPS_PROXY", "https_proxy"] {
        if let Ok(v) = std::env::var(key) {
            let v = v.trim();
            if !v.is_empty() {
                return Some(v.to_string());
            }
        }
    }
    None
}

fn once(endpoint: &str, query: &str) -> Result<Vec<u8>, String> {
    let agent = build_agent();
    let response = agent
        .post(endpoint)
        .set("User-Agent", UA)
        .send_form(&[("data", query)])
        .map_err(|e| format!("{e}"))?;
    let mut bytes = Vec::new();
    response
        .into_reader()
        .take(MAX_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("读取路网响应失败：{e}"))?;
    Ok(bytes)
}

/// 解析并校验 Overpass XML。
pub fn parse(bytes: &[u8]) -> Result<RoadGraph, String> {
    if bytes.len() as u64 > MAX_BYTES {
        return Err("路网响应超过 24 MiB".into());
    }
    let xml = std::str::from_utf8(bytes).map_err(|_| "路网不是有效 XML")?;
    if !xml.contains("<osm") || !xml.contains("</osm>") || xml.contains("<remark") {
        return Err("路网服务返回错误或不完整数据，请稍后重试".into());
    }
    let net = route_planner::load_osm(bytes)?;
    if net.graph.edge_count() == 0
        || net.graph.node_weights().any(|n| {
            !n.coord.lon.is_finite()
                || !n.coord.lat.is_finite()
                || n.coord.lon.abs() > 180.0
                || n.coord.lat.abs() > 90.0
        })
    {
        return Err("该区域没有可用的 OSM 道路数据，可手工导入完整路网".into());
    }
    Ok(net)
}

#[derive(serde::Serialize, serde::Deserialize)]
struct Meta {
    key: String,
    loaded_ms: i64,
    #[serde(default)]
    school: String,
    #[serde(default)]
    fences: Vec<Vec<(f64, f64)>>,
    #[serde(default)]
    points: Vec<(f64, f64)>,
}

/// 缓存键：范围到厘米级精度归一化后取 sha256，保证 JSON 浮点往返后仍稳定。
pub fn scope_key(fences: &[Vec<(f64, f64)>], points: &[(f64, f64)]) -> String {
    let f: Vec<Vec<(i64, i64)>> = fences
        .iter()
        .map(|r| {
            r.iter()
                .map(|&(a, b)| ((a * 1e7).round() as i64, (b * 1e7).round() as i64))
                .collect()
        })
        .collect();
    let p: Vec<(i64, i64)> = points
        .iter()
        .map(|&(a, b)| ((a * 1e7).round() as i64, (b * 1e7).round() as i64))
        .collect();
    format!("{:x}", Sha256::digest(serde_json::to_vec(&(f, p)).unwrap()))
}

/// 路网缓存目录。
pub fn dir() -> PathBuf {
    crate::platform::data_dir().join("osm_auto")
}

fn meta_path(dir: &Path, key: &str) -> Result<PathBuf, String> {
    if key.len() != 64 || !key.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("无效缓存标识".into());
    }
    Ok(dir.join(format!("{key}.osm.json")))
}

/// 列出缓存条目（按下载时间倒序）。
pub fn entries() -> Vec<(String, i64, u64)> {
    entries_in(&dir())
}

fn entries_in(dir: &Path) -> Vec<(String, i64, u64)> {
    let Ok(files) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for f in files.flatten() {
        let p = f.path();
        let Some(key) = p
            .file_stem()
            .and_then(|s| s.to_str())
            .map(|s| s.trim_end_matches(".osm"))
            .map(str::to_owned)
        else {
            continue;
        };
        let Ok(text) = std::fs::read_to_string(&p) else {
            continue;
        };
        let Ok(meta) = serde_json::from_str::<Meta>(&text) else {
            continue;
        };
        let bytes = std::fs::metadata(dir.join(format!("{key}.osm")))
            .map(|m| m.len())
            .unwrap_or(0);
        if meta.key == key {
            out.push((key, meta.loaded_ms, bytes));
        }
    }
    out.sort_by(|a, b| b.1.cmp(&a.1));
    out
}

/// 删除某条缓存（.osm 与 .json 一起删）。
pub fn clear_cache(key: &str) -> Result<(), String> {
    let dir = dir();
    let meta = meta_path(&dir, key)?;
    let _ = std::fs::remove_file(dir.join(format!("{key}.osm")));
    std::fs::remove_file(meta).map_err(|e| e.to_string())
}

/// 命中条件：标识一致 + 7 天内 + .osm 本体还在。
fn cached_osm(dir: &Path, key: &str, now: i64) -> Option<PathBuf> {
    let meta_file = meta_path(dir, key).ok()?;
    let text = std::fs::read_to_string(&meta_file).ok()?;
    let meta = serde_json::from_str::<Meta>(&text).ok()?;
    if meta.key != key || !(0..TTL_MS).contains(&(now - meta.loaded_ms)) {
        return None;
    }
    let osm = dir.join(format!("{key}.osm"));
    if osm.is_file() {
        Some(osm)
    } else {
        None
    }
}

/// 取当前学校的路网文件，命中缓存则不联网。
///
/// 返回可直接交给 `load_network_path` / 手工导入的 `.osm` 绝对路径。
pub fn fetch_osm(
    fences: &[Vec<(f64, f64)>],
    points: &[(f64, f64)],
    refresh: bool,
    log: &mut dyn FnMut(&str),
) -> Result<PathBuf, String> {
    fetch_osm_in(&dir(), fences, points, refresh, crate::crypto::envelope::now_ms(), log)
}

fn fetch_osm_in(
    dir: &Path,
    fences: &[Vec<(f64, f64)>],
    points: &[(f64, f64)],
    refresh: bool,
    now: i64,
    log: &mut dyn FnMut(&str),
) -> Result<PathBuf, String> {
    if fences.is_empty() && points.is_empty() {
        return Err("没有围栏也没有点位，无法确定下载范围".into());
    }
    let key = scope_key(fences, points);
    let osm = dir.join(format!("{key}.osm"));

    // 1. 缓存：7 天内直接复用，不联网。
    if !refresh {
        if let Some(p) = cached_osm(dir, &key, now) {
            log("[road] 命中路网缓存，直接使用");
            return Ok(p);
        }
    }

    // 2. 下载。
    let bbox = if fences.is_empty() {
        query_points_bbox(points)?
    } else {
        query_fences_bbox(fences)?
    };
    let q = build_query(bbox);
    let (km_y, km_x) = span_km(bbox);
    log(&format!(
        "[road] 范围约 {km_x:.1} × {km_y:.1} km，正在下载 OpenStreetMap 路网…"
    ));
    let bytes = download(&q, log)?;
    let net = parse(&bytes)?;
    log(&format!(
        "[road] 节点 {} · 有向边 {} · 建筑 {}",
        net.graph.node_count(),
        net.graph.edge_count(),
        net.buildings.len()
    ));

    // 3. 落盘：先写临时文件再 rename，避免并发/中断留下半个文件。
    std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    let write = |name: &str, body: &[u8]| -> Result<(), String> {
        let tmp = dir.join(format!(".{name}.tmp"));
        std::fs::write(&tmp, body).map_err(|e| e.to_string())?;
        std::fs::rename(&tmp, dir.join(name)).map_err(|e| e.to_string())
    };
    write(&format!("{key}.osm"), &bytes)?;
    let meta = Meta {
        key: key.clone(),
        loaded_ms: now,
        school: String::new(),
        fences: fences.to_vec(),
        points: points.to_vec(),
    };
    let body = serde_json::to_vec(&meta).map_err(|e| e.to_string())?;
    write(&format!("{key}.osm.json"), &body)?;
    Ok(osm)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(c: (f64, f64), d: f64) -> Vec<(f64, f64)> {
        vec![
            (c.0 - d, c.1 - d),
            (c.0 - d, c.1 + d),
            (c.0 + d, c.1 + d),
            (c.0 + d, c.1 - d),
        ]
    }

    fn temp_dir(tag: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("neko-sr-{tag}"));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    #[test]
    fn fences_must_be_usable() {
        assert!(validate_fences(&[]).is_err());
        // 退化的环（三点共线）面积为 0
        let flat = vec![vec![(30.0, 120.0), (30.0, 120.1), (30.0, 120.2)]];
        assert!(validate_fences(&flat).is_err());
        let bad = vec![vec![(91.0, 120.0), (30.0, 120.1), (30.0, 120.2)]];
        assert!(validate_fences(&bad).is_err());
        assert!(validate_fences(&[ring((30.0, 120.0), 0.002)]).is_ok());
    }

    #[test]
    fn to_wgs_inverts_bd09() {
        // 先正向得到 BD-09，再反解，应回到 WGS-84 原点
        for (lat, lon) in [(30.0, 120.0), (39.9042, 116.4074), (23.12, 113.25)] {
            let (b_lat, b_lon) = crate::track::geom::wgs84_to_bd09(lat, lon);
            let (w_lat, w_lon) = to_wgs(b_lat, b_lon);
            assert!((w_lat - lat).abs() < 1e-7, "lat {w_lat} vs {lat}");
            assert!((w_lon - lon).abs() < 1e-7, "lon {w_lon} vs {lon}");
        }
    }

    #[test]
    fn bbox_rejects_huge_span() {
        let wide = vec![
            (20.0, 100.0),
            (20.0, 140.0),
            (50.0, 140.0),
            (50.0, 100.0),
        ];
        assert!(bbox_of(&wide, 0.0).is_err());
        let small = ring((30.0, 120.0), 0.002);
        let (s, w, n, e) = bbox_of(&small, 0.001).unwrap();
        assert!(n > s && e > w);
        // 外扩后跨度仍在 0.15/0.2 以内
        assert!(n - s < MAX_SPAN_LAT);
        assert!(e - w < MAX_SPAN_LON);
    }

    #[test]
    fn query_carries_bbox_and_timeout() {
        let q = build_query((1.0, 2.0, 3.0, 4.0));
        assert!(q.contains("[out:xml][timeout:25];"));
        assert!(q.contains("way[highway](1.0000000,2.0000000,3.0000000,4.0000000)"));
        assert!(q.contains("way[building]"));
        assert!(q.contains("relation[building]"));
        assert!(q.contains("(._;>;);out body;"));
    }

    #[test]
    fn scope_key_is_stable_and_sensitive() {
        let a = ring((30.0, 120.0), 0.002);
        // JSON 浮点往返的误差量级（1e-8 度，约 1 毫米）内必须稳定，否则缓存永不命中
        let b = ring((30.00000001, 120.0), 0.002);
        assert_eq!(scope_key(&[a.clone()], &[]), scope_key(&[b], &[]));
        // 换到另一个城市（约 55 公里外）必须换桶
        let c = ring((30.5, 120.0), 0.002);
        assert_ne!(scope_key(&[a], &[]), scope_key(&[c], &[]));
        assert_eq!(scope_key(&[], &[(1.0, 2.0)]).len(), 64);
    }

    #[test]
    fn parse_rejects_non_osm() {
        assert!(parse(b"not xml at all").is_err());
        assert!(parse(b"<osm><remark>x</remark></osm>").is_err());
    }

    /// 预置一条缓存，age_ms 让它看起来有多旧。
    fn seed_cache(dir: &Path, fences: &[Vec<(f64, f64)>], age_ms: i64) -> String {
        let key = scope_key(fences, &[]);
        std::fs::write(dir.join(format!("{key}.osm")), b"<osm></osm>").unwrap();
        let meta = Meta {
            key: key.clone(),
            loaded_ms: crate::crypto::envelope::now_ms() - age_ms,
            school: String::new(),
            fences: fences.to_vec(),
            points: vec![],
        };
        std::fs::write(
            dir.join(format!("{key}.osm.json")),
            serde_json::to_vec(&meta).unwrap(),
        )
        .unwrap();
        key
    }

    #[test]
    fn cache_hit_returns_path_without_network() {
        let dir = temp_dir("cache");
        let fences = [ring((30.0, 120.0), 0.002)];
        let key = seed_cache(&dir, &fences, 0);
        assert_eq!(entries_in(&dir).len(), 1);

        let mut logs: Vec<String> = Vec::new();
        let got = fetch_osm_in(
            &dir,
            &fences,
            &[],
            false,
            crate::crypto::envelope::now_ms(),
            &mut |s| logs.push(s.to_string()),
        )
        .unwrap();
        assert_eq!(got, dir.join(format!("{key}.osm")));
        assert!(logs.iter().any(|l| l.contains("命中路网缓存")));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn expired_or_broken_cache_is_not_used() {
        let dir = temp_dir("stale");
        // 落盘时间推到 14 天前：远超 7 天 TTL，毫秒级时钟抖动不会影响结论
        let key = seed_cache(&dir, &[ring((30.0, 120.0), 0.002)], TTL_MS * 2);
        let now = crate::crypto::envelope::now_ms();
        assert!(cached_osm(&dir, &key, now).is_none(), "过期缓存不应命中");

        // 新鲜缓存应当命中；删掉 .osm 本体后不再命中
        let key2 = seed_cache(&dir, &[ring((31.0, 121.0), 0.002)], 0);
        let now2 = crate::crypto::envelope::now_ms();
        assert!(cached_osm(&dir, &key2, now2).is_some(), "新鲜缓存应命中");
        std::fs::remove_file(dir.join(format!("{key2}.osm"))).unwrap();
        assert!(
            cached_osm(&dir, &key2, now2).is_none(),
            "缺少 .osm 本体不应命中"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_scope_is_reported() {
        let mut logs: Vec<String> = Vec::new();
        let err = fetch_osm_in(&temp_dir("empty"), &[], &[], false, 0, &mut |s| {
            logs.push(s.to_string())
        })
        .unwrap_err();
        assert!(err.contains("无法确定下载范围"));
    }
}
