use crate::debugger::Debugger;
use crate::{log, worker_scope};
use core::cell::RefCell;
use gloo_timers::callback::Interval;
use js_sys::Array;
use js_sys::{Object, Reflect, Uint16Array, Uint8Array};
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::{JsCast, JsValue};

fn prop_string(obj: &JsValue, prop: &str) -> String {
    Reflect::get(obj, &JsValue::from_str(prop))
        .expect_throw(&format!("Property {} was missing on object", prop))
        .as_string()
        .expect_throw(&format!("Property {} was incorrect type on object", prop))
}

fn prop_object<T: JsCast>(obj: &JsValue, prop: &str) -> T {
    Reflect::get(obj, &JsValue::from_str(prop))
        .expect_throw(&format!("Property {} was missing on object", prop))
        .dyn_into::<T>()
        .expect_throw(&format!("Property {} was incorrect type on object", prop))
}

struct ObjectBuilder {
    obj: Object,
}

impl ObjectBuilder {
    fn new() -> ObjectBuilder {
        let obj = Object::new();
        ObjectBuilder { obj }
    }
    fn string(self, prop: &str, val: &str) -> Self {
        Reflect::set(&self.obj, &JsValue::from_str(prop), &JsValue::from_str(val)).unwrap_throw();
        self
    }
}

struct MessageBuilder {
    msg: Object,
    transfer: Array,
}

impl MessageBuilder {
    fn new(message_type: &str) -> MessageBuilder {
        let msg = Object::new();
        let transfer = Array::new();
        Reflect::set(&msg, &JsValue::from_str("type"), &JsValue::from_str(message_type)).unwrap_throw();
        MessageBuilder { msg, transfer }
    }

    // fn string(self, prop: &str, val: &str) -> Self {
    //     Reflect::set(&self.msg, &JsValue::from_str(prop), &JsValue::from_str(val)).unwrap_throw();
    //     self
    // }

    fn val<T: AsRef<JsValue>>(self, prop: &str, val: T) -> Self {
        Reflect::set(&self.msg, &JsValue::from_str(prop), val.as_ref()).unwrap_throw();
        self
    }

    fn obj<F>(self, prop: &str, f: F) -> Self
    where
        F: FnOnce(ObjectBuilder) -> ObjectBuilder,
    {
        let obj = f(ObjectBuilder::new());
        Reflect::set(&self.msg, &JsValue::from_str(prop), &obj.obj).unwrap_throw();
        self
    }

    fn transfer<T: AsRef<JsValue>>(self, transfer: T) -> Self {
        self.transfer.push(transfer.as_ref());
        self
    }

    fn send(self) {
        worker_scope()
            .post_message_with_transfer(&self.msg, &self.transfer)
            .unwrap_throw();
    }
}

pub struct Worker {
    debugger: Rc<RefCell<Option<Debugger>>>,
    emulation_tick: Option<Interval>,
}

impl Worker {
    pub fn new() -> Worker {
        Worker {
            debugger: Rc::new(RefCell::new(None)),
            emulation_tick: None,
        }
    }

    pub fn init(&mut self) {
        MessageBuilder::new("init").send();
    }

    pub fn on_message(&mut self, message: JsValue) {
        let message_type: String = prop_string(&message, "type");

        match message_type.as_str() {
            "load" => self.load_rom(prop_object(&message, "rom")),
            "start" => self.start_emulation(),
            "pause" => self.pause_emulation(),
            "refresh" => self.refresh_state(prop_object(&message, "framebuffer"), prop_object(&message, "memory")),
            "keydown" => self.handle_keypress(&prop_string(&message, "key"), true),
            "keyup" => self.handle_keypress(&prop_string(&message, "key"), false),
            _ => log!("Received unhandled message {}", message_type),
        }
    }

    fn load_rom(&mut self, data: Uint8Array) {
        // If we were running another ROM already, make sure we clean up old state.
        self.emulation_tick = None;

        let mut rom = vec![0; data.length() as usize];
        data.copy_to(&mut rom);
        let debugger = Debugger::new(&rom);

        MessageBuilder::new("loaded")
            .obj("rom", |obj| {
                obj.string("hash", &debugger.rom_hash())
                    .string("title", &debugger.rom_title())
            })
            .send();

        *self.debugger.borrow_mut() = Some(debugger);
        //   await set(`${rom_hash}_rom`, message.rom);
    }

    fn start_emulation(&mut self) {
        if self.debugger.borrow().is_none() || self.emulation_tick.is_some() {
            return;
        }

        let debugger = self.debugger.clone();
        self.emulation_tick = Some(Interval::new(2, move || {
            debugger.borrow_mut().as_mut().unwrap().run(2000.0);
        }));

        MessageBuilder::new("running").send();
    }

    fn pause_emulation(&mut self) {
        self.emulation_tick.take();
        MessageBuilder::new("paused").send();
    }

    fn refresh_state(&self, framebuffer: Uint16Array, mem: Uint8Array) {
        let debugger = self.debugger.borrow();
        let debugger = debugger.as_ref().unwrap();

        framebuffer.set(unsafe { &Uint16Array::view(debugger.framebuffer()) }, 0);
        mem.set(unsafe { &Uint8Array::view(debugger.memory()) }, 0);

        MessageBuilder::new("state")
            .val("framebuffer", &framebuffer)
            .val("memory", &mem)
            .val("cpu", &debugger.cpu_state())
            .transfer(framebuffer.buffer())
            .transfer(mem.buffer())
            .send();
    }

    fn handle_keypress(&self, key: &str, pressed: bool) {
        self.debugger
            .borrow_mut()
            .as_mut()
            .unwrap()
            .set_joypad_state(key, pressed);
    }
}
