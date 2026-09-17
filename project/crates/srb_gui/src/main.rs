fn main() -> eframe::Result<()> {
    tracing_subscriber::fmt::init();

    let icon = image::load_from_memory_with_format(
        include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../docs/theme/favicon.png"
        )),
        image::ImageFormat::Png,
    )
    .unwrap()
    .to_rgba8();
    let (icon_width, icon_height) = icon.dimensions();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_icon(egui::IconData {
            rgba: icon.into_raw(),
            width: icon_width,
            height: icon_height,
        }),
        ..Default::default()
    };

    eframe::run_native(
        "Space Robotics Bench",
        native_options,
        Box::new(|cc| Ok(Box::new(srb_gui::App::new(cc)))),
    )
}
