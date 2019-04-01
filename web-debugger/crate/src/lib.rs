mod debugger;
mod disassembler;
mod worker;

use crate::worker::Worker;
use cfg_if::cfg_if;
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{DedicatedWorkerGlobalScope, MessageEvent};

#[macro_export]
macro_rules! log {
    ( $( $args:tt )* ) => {
        web_sys::console::log_1(&format!( $( $args )* ).into());
    }
}

cfg_if! {
    // When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
    // allocator.
    if #[cfg(feature = "wee_alloc")] {
        extern crate wee_alloc;
        #[global_allocator]
        static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;
    }
}

cfg_if! {
    // When the `console_error_panic_hook` feature is enabled, we can call the
    // `set_panic_hook` function at least once during initialization, and then
    // we will get better error messages if our code ever panics.
    //
    // For more details see
    // https://github.com/rustwasm/console_error_panic_hook#readme
    if #[cfg(feature = "console_error_panic_hook")] {
        extern crate console_error_panic_hook;
        pub use self::console_error_panic_hook::set_once as set_panic_hook;
    } else {
        #[inline]
        pub fn set_panic_hook() {}
    }
}

fn worker_scope() -> DedicatedWorkerGlobalScope {
    js_sys::global()
        .dyn_into::<DedicatedWorkerGlobalScope>()
        .expect("global worker scope does not exist")
}

#[wasm_bindgen]
pub fn run() {
    set_panic_hook();

    let worker = Rc::new(RefCell::new(Worker::new()));
    worker.borrow_mut().init();

    let message_handler = Closure::wrap(Box::new(move |event: MessageEvent| {
        worker.borrow_mut().on_message(event.data());
    }) as Box<FnMut(MessageEvent)>);
    worker_scope().set_onmessage(Some(message_handler.as_ref().unchecked_ref()));
    message_handler.forget();
}
