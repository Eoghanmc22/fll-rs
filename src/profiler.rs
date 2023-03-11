use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::time::{Duration, Instant};

// TODO Actually use this

#[derive(Clone)]
pub struct Profiler {
    stack: Vec<Frame>,
    root: Node,
}

#[derive(Clone)]
struct Frame {
    start: Instant,
    name: String,
}

#[derive(Clone)]
struct ElapsedFrame {
    duration: Duration,
    name: String,
    count: u32,
}

#[derive(Clone)]
struct Node {
    frame: ElapsedFrame,
    children: HashMap<String, Node>,
}

impl Frame {
    fn elapsed(self) -> ElapsedFrame {
        ElapsedFrame {
            duration: self.start.elapsed(),
            name: self.name,
            count: 1,
        }
    }
}

impl ElapsedFrame {
    pub fn with_name(name: &str) -> ElapsedFrame {
        ElapsedFrame {
            duration: Duration::from_secs(0),
            name: name.to_owned(),
            count: 0,
        }
    }
}

impl Node {
    pub fn with_name(name: &str) -> Node {
        Node {
            frame: ElapsedFrame::with_name(name),
            children: HashMap::new(),
        }
    }

    pub fn get(&mut self, name: &str) -> &mut Node {
        if let Entry::Vacant(vacant) = self.children.entry(name.to_owned()) {
            vacant.insert(Node::with_name(name));
        }

        // Safety we just checked it exists
        self.children.get_mut(name).unwrap()
    }

    pub fn insert_frame(&mut self, frame: &ElapsedFrame) {
        let old = self
            .children
            .remove(&frame.name)
            .unwrap_or_else(|| Node::with_name(&frame.name));
        let new = Node {
            frame: ElapsedFrame {
                duration: old.frame.duration + frame.duration,
                count: old.frame.count + frame.count,
                ..old.frame
            },
            ..old
        };

        self.children.insert(frame.name.to_owned(), new);
    }
}

impl Profiler {
    pub fn new() -> Self {
        Self {
            stack: vec![],
            root: Node::with_name("Root"),
        }
    }

    pub fn push(&mut self, name: &str) {
        self.stack.push(Frame {
            start: Instant::now(),
            name: name.to_owned(),
        });
    }

    pub fn pop_named(&mut self, expected_name: &str) {
        if let Some(frame) = self.stack.pop() {
            if !expected_name.is_empty() && frame.name != expected_name {
                panic!(
                    "profiler over pushed, got {} expected {}",
                    frame.name, expected_name
                );
            }

            let elapsed_frame = frame.elapsed();
            let mut node = &mut self.root;

            for frame in self.stack.iter() {
                node = node.get(&frame.name);
            }

            node.insert_frame(&elapsed_frame);
        } else {
            panic!("profiler over pop-ed");
        }
    }

    pub fn pop(&mut self) {
        self.pop_named("");
    }

    pub fn report(&self) {
        Profiler::report_0(&self.root, 0);
    }

    fn report_0(node: &Node, depth: usize) {
        let x = node.frame.duration.as_secs_f32() * 1000.0;
        eprintln!(
            "{}{} took {} ms over {} calls (avg. {} ms)",
            "\t".repeat(depth),
            node.frame.name,
            x,
            node.frame.count,
            x / node.frame.count as f32
        );

        for entry in node.children.iter() {
            Profiler::report_0(entry.1, depth + 1);
        }
    }

    pub fn merge(&mut self, other: &Profiler) {
        Profiler::merge_0(&mut self.root, &other.root);
    }

    fn merge_0(this: &mut Node, other: &Node) {
        this.insert_frame(&other.frame);

        for (name, node) in other.children.iter() {
            match this.children.entry(name.to_owned()) {
                Entry::Occupied(mut occupied) => {
                    Profiler::merge_0(occupied.get_mut(), node);
                }
                Entry::Vacant(vacant) => {
                    vacant.insert(node.clone());
                }
            }
        }
    }
}
