//! 「路网」导入页：选择 OSM 文件 → 后台解析 → 画布预览 + 统计。

use eframe::egui;
use std::sync::Arc;

use route_planner::{Coord, RoadGraph};

use super::map::{collect_bounds, MapState};
use super::{theme, App};

#[derive(Default)]
pub struct OsmPage {
    pub path: String,
    pub busy: bool,
    pub status: String,
    pub map: MapState,
    pub fitted: bool,
}

fn ring_to_pts(ring: &[Coord]) -> Vec<(f64, f64)> {
    ring.iter().map(|c| (c.lat, c.lon)).collect()
}

impl App {
    pub fn draw_osm_page(&mut self, ui: &mut egui::Ui) {
        let mut load_now = false;
        let mut auto_now = false;
        let mut auto_refresh = false;
        let mut clear_now = false;

        ui.horizontal(|ui| {
            ui.label("OSM 文件：");
            ui.add_sized(
                [340.0, 26.0],
                egui::TextEdit::singleline(&mut self.osm_page.path)
                    .hint_text("拖入 .osm 文件或填写路径"),
            );
            #[cfg(not(target_os = "android"))]
            if ui.button("浏览…").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("OSM", &["osm", "xml", "pbf"])
                    .pick_file()
                {
                    self.osm_page.path = p.display().to_string();
                }
            }
            let can_load = !self.osm_page.path.is_empty() && !self.osm_page.busy;
            if ui
                .add_enabled(can_load, theme::primary_btn("加载路网"))
                .clicked()
            {
                load_now = true;
            }
            // 自动获取：拿已缓存的学校围栏向 Overpass 拉路网，落盘后走同一条加载流程。
            if ui
                .add_enabled(!self.osm_page.busy, theme::primary_btn("自动获取路网"))
                .clicked()
            {
                auto_now = true;
            }
            if ui
                .add_enabled(!self.osm_page.busy, egui::Button::new("重新下载"))
                .on_hover_text("忽略 7 天缓存，重新下载当前区域路网")
                .clicked()
            {
                auto_now = true;
                auto_refresh = true;
            }
            if ui
                .add_enabled(!self.osm_page.busy, egui::Button::new("清空缓存"))
                .on_hover_text("删除本目录下已缓存的区域路网，下次会重新下载")
                .clicked()
            {
                clear_now = true;
            }
        });

        // 拖拽导入
        let dropped: Vec<String> = ui.ctx().input(|i| {
            i.raw
                .dropped_files
                .iter()
                .filter_map(|f| f.path.as_ref().map(|p| p.display().to_string()))
                .collect()
        });
        if let Some(p) = dropped.into_iter().find(|p| !p.is_empty()) {
            self.osm_page.path = p;
            if !self.osm_page.busy {
                load_now = true;
            }
        }

        if !self.osm_page.status.is_empty() {
            ui.colored_label(theme::plain(), &self.osm_page.status);
        }
        let cached = crate::api::school_roads::entries().len();
        if cached > 0 {
            ui.label(format!("已缓存 {cached} 个区域路网（7 天内自动复用）"));
        }

        // 统计
        if let Some(net) = &self.network {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "节点 {} · 有向边 {} · 建筑 {}",
                    net.graph.node_count(),
                    net.graph.edge_count(),
                    net.buildings.len()
                ));
                if let Ok(meta) = std::fs::metadata(&self.osm_page.path) {
                    ui.label(format!("· 文件 {:.1} KB", meta.len() as f64 / 1024.0));
                }
            });
        }

        ui.add_space(4.0);
        if self.network.is_some() {
            super::map::legend(
                ui,
                &[
                    ("道路", egui::Color32::from_rgb(190, 200, 196)),
                    ("建筑", egui::Color32::from_rgb(214, 182, 158)),
                ],
            );
        }
        let rect = ui.available_rect_before_wrap();
        let (response, painter) = ui.allocate_painter(
            egui::Vec2::new(rect.width().max(200.0), rect.height().max(220.0)),
            egui::Sense::drag(),
        );
        let canvas = response.rect;

        if let Some(net) = &self.network {
            if !self.osm_page.fitted {
                let mut bounds: Vec<(f64, f64)> = collect_bounds(
                    &net.edges_deg()
                        .iter()
                        .map(|e| ring_to_pts(e))
                        .collect::<Vec<_>>(),
                );
                for b in &net.buildings {
                    bounds.extend(ring_to_pts(b));
                }
                self.osm_page.map.fit(canvas, &bounds);
                self.osm_page.fitted = true;
            }
        }

        painter.rect_filled(
            canvas,
            egui::Rounding::ZERO,
            egui::Color32::from_rgb(250, 252, 251),
        );
        if let Some(net) = &self.network {
            let road_color = egui::Color32::from_rgb(190, 200, 196);
            for e in net.edges_deg() {
                self.osm_page.map.draw_polyline(
                    &painter,
                    canvas,
                    &ring_to_pts(&e),
                    road_color,
                    1.0,
                );
            }
            let bld_color = egui::Color32::from_rgb(214, 182, 158);
            for b in &net.buildings {
                self.osm_page
                    .map
                    .draw_polygon(&painter, canvas, &ring_to_pts(b), bld_color, 1.0);
            }
        } else {
            painter.text(
                canvas.center(),
                egui::Align2::CENTER_CENTER,
                "加载 OSM 路网后可预览道路与建筑",
                egui::FontId::proportional(14.0),
                egui::Color32::from_rgb(150, 160, 156),
            );
        }

        self.osm_page.map.interact(ui, canvas);

        if load_now {
            let path = self.osm_page.path.clone();
            self.load_osm_async(path);
        }
        if auto_now {
            self.fetch_osm_auto(auto_refresh);
        }
        if clear_now {
            let mut n = 0usize;
            for (key, _, _) in crate::api::school_roads::entries() {
                if let Err(e) = crate::api::school_roads::clear_cache(&key) {
                    self.log.push(&format!("[road] 清除缓存失败：{e}"));
                } else {
                    n += 1;
                }
            }
            self.osm_page.status = format!("已清空 {n} 个区域路网缓存");
        }
    }

    /// 后台下载学校区域路网，拿到 .osm 路径后回传主线程走普通加载流程。
    ///
    /// 范围和点位都取本地缓存，不额外发业务请求；下载本身只访问公开的 Overpass。
    pub fn fetch_osm_auto(&mut self, refresh: bool) {
        self.osm_page.busy = true;
        self.osm_page.status = "正在获取学校区域路网…".to_string();
        let tx = self.osm_auto_tx.clone();
        let log_tx = self.tx.clone();
        let fences = crate::api::model::load_fence_cache().unwrap_or_default();
        let points = self
            .identity
            .anchor_coordinate()
            .ok()
            .and_then(|a| crate::api::model::load_points_cache_for(a))
            .map(|(_, pts)| crate::api::points::points_bd(&pts))
            .unwrap_or_default();
        std::thread::spawn(move || {
            let res =
                crate::api::school_roads::fetch_osm(&fences, &points, refresh, &mut |s| {
                    let _ = log_tx.send(s.to_string());
                })
                .map(|p| p.to_string_lossy().to_string());
            let _ = tx.send(res);
        });
    }

    /// 后台线程加载 OSM 路网，成功后经 `net_rx` 回传。
    pub fn load_osm_async(&mut self, path: String) {
        self.osm_page.path = path.clone();
        self.osm_page.busy = true;
        self.osm_page.status = format!("加载中：{path} …");
        self.config.osm_path = path.clone();
        let _ = crate::api::model::save_config(&self.config);
        let tx = self.net_tx.clone();
        std::thread::spawn(move || {
            let res = (|| -> Result<Arc<RoadGraph>, String> {
                let bytes = std::fs::read(&path).map_err(|e| format!("读取失败: {e}"))?;
                let net = route_planner::load_osm(&bytes)?;
                Ok(Arc::new(net))
            })();
            let _ = tx.send(res);
        });
    }
}
