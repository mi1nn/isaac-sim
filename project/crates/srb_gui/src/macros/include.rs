macro_rules! include_content_image {
    ($file:expr $(,)?) => {
        ::egui::ImageSource::Bytes {
            uri: ::std::borrow::Cow::Borrowed(concat!("bytes://", concat!("content/", $file))),
            bytes: ::egui::load::Bytes::Static(include_bytes!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/content/",
                $file
            ))),
        }
    };
}

macro_rules! include_assets_font {
    ($file:expr $(,)?) => {
        ::egui::FontData::from_static(include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/assets/fonts/",
            $file
        )))
    };
}

pub(crate) use {include_assets_font, include_content_image};
