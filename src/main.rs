#![cfg_attr(all(windows, not(debug_assertions)), windows_subsystem = "windows")]

mod protocol;
mod serial;
mod ui;

fn main() {
    dioxus::launch(ui::app);
}
