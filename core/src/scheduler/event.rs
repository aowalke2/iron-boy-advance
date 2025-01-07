use std::cmp::Ordering;

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum PpuEvent {
    None,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum ApuEvent {
    Sample,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum EventType {
    FrameComplete,
    Ppu(PpuEvent),
    Apu(ApuEvent),
}

pub type FutureEvent = (EventType, usize);

#[derive(Debug, Clone, Eq)]
pub struct Event {
    event_type: EventType,
    time: usize,
}

impl Event {
    pub fn new(event_type: EventType, time: usize) -> Event {
        Event { event_type, time }
    }

    pub fn event_type(&self) -> EventType {
        self.event_type
    }

    pub fn time(&self) -> usize {
        self.time
    }
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        other.time.cmp(&self.time)
    }
}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.time.partial_cmp(&self.time)
    }

    fn lt(&self, other: &Self) -> bool {
        other.time < self.time
    }

    fn le(&self, other: &Self) -> bool {
        other.time <= self.time
    }

    fn gt(&self, other: &Self) -> bool {
        other.time > self.time
    }

    fn ge(&self, other: &Self) -> bool {
        other.time >= self.time
    }
}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
    }
}
