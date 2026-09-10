fn main() -> eframe::Result<()> {
    let icon = eframe::icon_data::from_png_bytes(include_bytes!("../assets/squid.png"))
        .expect("アイコン画像 assets/squid.png を読み込めない");
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([800.0, 600.0])
            .with_icon(icon),
        ..Default::default()
    };
    eframe::run_native(
        "Squid-n",
        options,
        Box::new(|cc| {
            squid_n_app::app::install_japanese_fonts(&cc.egui_ctx);
            squid_n_app::theme::apply_theme(&cc.egui_ctx);
            let mut app = squid_n_app::app::App::default();
            app.load_model(squid_n_core::model::Model::with_default_load_cases());
            Ok(Box::new(app))
        }),
    )
}
