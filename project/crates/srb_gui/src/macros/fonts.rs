macro_rules! generate_font_definitions {
    {$($family_name:literal $(as $font_family:ident)? [$($name:literal),+ $(,)?]),* $(,)?} => {{
        let mut font_data = ::std::collections::BTreeMap::new();
        let mut families = ::std::collections::BTreeMap::new();
        let mut monospace_fonts = ::std::vec::Vec::new();
        let mut proportional_fonts = ::std::vec::Vec::new();
        $(
            $crate::macros::generate_font_definitions!(@insert_family |font_data, families, monospace_fonts, proportional_fonts| $family_name $(as $font_family)? [$($name),+]);
        )*
        families.insert(
            ::egui::FontFamily::Monospace,
            monospace_fonts,
        );
        families.insert(
            ::egui::FontFamily::Proportional,
            proportional_fonts,
        );
        ::egui::FontDefinitions {
            font_data,
            families,
        }
    }};
    (@insert_family |$font_data:ident, $families:ident, $monospace_fonts:ident, $proportional_fonts:ident| $family_name:literal as Monospace [$($name:literal),+ $(,)?]) => {
        $crate::macros::generate_font_definitions!(@insert_family |$font_data, $families, $monospace_fonts, $proportional_fonts| $family_name [$($name),+]);
        $monospace_fonts.extend_from_slice(&[$($name.to_owned()),+]);
    };
    (@insert_family |$font_data:ident, $families:ident, $monospace_fonts:ident, $proportional_fonts:ident| $family_name:literal as Proportional [$($name:literal),+ $(,)?]) => {
        $crate::macros::generate_font_definitions!(@insert_family |$font_data, $families, $monospace_fonts, $proportional_fonts| $family_name [$($name),+]);
        $proportional_fonts.extend_from_slice(&[$($name.to_owned()),+]);
    };
    (@insert_family |$font_data:ident, $families:ident, $monospace_fonts:ident, $proportional_fonts:ident| $family_name:literal [$($name:literal),+ $(,)?]) => {
        $(
            $crate::macros::generate_font_definitions!(@insert_data |$font_data| $name);
        )*
        $families.insert(
            ::egui::FontFamily::Name($family_name.into()),
            vec![$($name.to_owned()),+],
        );
    };
    (@insert_data |$font_data:ident| $name:literal) => {
        $font_data.insert(
            $name.to_owned(),
            $crate::macros::include_assets_font!($name).into(),
        );
    };
}
pub(crate) use generate_font_definitions;
