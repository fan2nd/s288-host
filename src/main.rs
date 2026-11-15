mod protocol;
mod serial;
mod ui;

fn main() {
    dioxus::launch(ui::app);
}
