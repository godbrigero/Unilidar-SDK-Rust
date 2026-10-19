use crate::lidar::{IDLE_SLEEP, Lidar};
use crate::types::Event;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread::JoinHandle;
use std::time::Duration;

/// Background threads that each own one [`Lidar`] and push its events into a channel.
struct Workers {
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<Lidar>>,
}

impl Workers {
    fn spawn<K, T>(
        lidars: Vec<(Lidar, K)>,
        tag: fn(K, Event) -> T,
        send: impl Fn(T) -> bool + Clone + Send + 'static,
    ) -> Self
    where
        K: Clone + Send + 'static,
        T: Send + 'static,
    {
        let stop = Arc::new(AtomicBool::new(false));
        let threads = lidars
            .into_iter()
            .map(|(mut lidar, key)| {
                let stop = stop.clone();
                let send = send.clone();
                std::thread::Builder::new()
                    .name("unitree-lidar".into())
                    .spawn(move || {
                        while !stop.load(Ordering::Relaxed) {
                            match lidar.try_next_event() {
                                Some(event) => {
                                    if !send(tag(key.clone(), event)) {
                                        // Receiver is gone, nobody is listening anymore
                                        break;
                                    }
                                }
                                None => std::thread::sleep(IDLE_SLEEP),
                            }
                        }
                        lidar
                    })
                    .expect("failed to spawn lidar thread")
            })
            .collect();
        Self { stop, threads }
    }

    fn join(&mut self) -> Vec<Lidar> {
        self.stop.store(true, Ordering::Relaxed);
        self.threads
            .drain(..)
            .map(|thread| thread.join().unwrap_or_else(|panic| std::panic::resume_unwind(panic)))
            .collect()
    }
}

impl Drop for Workers {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

/// Receives events from lidars running on background threads.
///
/// Created by [`Lidar::spawn`] (items are [`Event`]) or [`LidarGroup::spawn`]
/// (items are `(key, Event)`). Dropping the handle stops the threads and the lidars.
///
/// The channel is unbounded, so keep up with the incoming data or the
/// backlog will grow.
pub struct LidarHandle<T> {
    rx: mpsc::Receiver<T>,
    workers: Workers,
}

impl<T: Send + 'static> LidarHandle<T> {
    pub(crate) fn spawn<K: Clone + Send + 'static>(
        lidars: Vec<(Lidar, K)>,
        tag: fn(K, Event) -> T,
    ) -> Self {
        let (tx, rx) = mpsc::channel();
        let workers = Workers::spawn(lidars, tag, move |item| tx.send(item).is_ok());
        Self { rx, workers }
    }
}

impl<T> LidarHandle<T> {
    /// Blocks until the next event arrives.
    pub fn recv(&self) -> Option<T> {
        self.rx.recv().ok()
    }

    /// Returns the next event if one is already waiting.
    pub fn try_recv(&self) -> Option<T> {
        self.rx.try_recv().ok()
    }

    /// Blocks for at most `timeout` waiting for the next event.
    pub fn recv_timeout(&self, timeout: Duration) -> Option<T> {
        self.rx.recv_timeout(timeout).ok()
    }

    /// Returns every event that is already waiting, without blocking.
    pub fn drain(&self) -> impl Iterator<Item = T> + '_ {
        self.rx.try_iter()
    }
}

impl<T> Iterator for LidarHandle<T> {
    type Item = T;

    fn next(&mut self) -> Option<T> {
        self.recv()
    }
}

impl LidarHandle<Event> {
    /// Stops the background thread and gives the lidar back.
    pub fn stop(self) -> Lidar {
        let mut workers = self.workers;
        workers.join().pop().expect("handle owns exactly one lidar")
    }
}

impl<K> LidarHandle<(K, Event)> {
    /// Stops all background threads and gives the lidars back, in the order they were added.
    pub fn stop(self) -> Vec<Lidar> {
        let mut workers = self.workers;
        workers.join()
    }
}

/// Like [`LidarHandle`], but receiving is `async` and it is a [`Stream`](futures_core::Stream).
/// It works with any async runtime.
#[cfg(feature = "async")]
pub struct AsyncLidarHandle<T> {
    rx: tokio::sync::mpsc::UnboundedReceiver<T>,
    workers: Workers,
}

#[cfg(feature = "async")]
impl<T: Send + 'static> AsyncLidarHandle<T> {
    pub(crate) fn spawn<K: Clone + Send + 'static>(
        lidars: Vec<(Lidar, K)>,
        tag: fn(K, Event) -> T,
    ) -> Self {
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        let workers = Workers::spawn(lidars, tag, move |item| tx.send(item).is_ok());
        Self { rx, workers }
    }
}

#[cfg(feature = "async")]
impl<T> AsyncLidarHandle<T> {
    /// Waits for the next event.
    pub async fn recv(&mut self) -> Option<T> {
        self.rx.recv().await
    }

    /// Returns the next event if one is already waiting.
    pub fn try_recv(&mut self) -> Option<T> {
        self.rx.try_recv().ok()
    }
}

#[cfg(feature = "async")]
impl<T> futures_core::Stream for AsyncLidarHandle<T> {
    type Item = T;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<T>> {
        self.rx.poll_recv(cx)
    }
}

#[cfg(feature = "async")]
impl AsyncLidarHandle<Event> {
    /// Stops the background thread and gives the lidar back.
    pub fn stop(self) -> Lidar {
        let mut workers = self.workers;
        workers.join().pop().expect("handle owns exactly one lidar")
    }
}

#[cfg(feature = "async")]
impl<K> AsyncLidarHandle<(K, Event)> {
    /// Stops all background threads and gives the lidars back, in the order they were added.
    pub fn stop(self) -> Vec<Lidar> {
        let mut workers = self.workers;
        workers.join()
    }
}

/// Several lidars read in parallel, with every event tagged by a key you choose.
///
/// ```no_run
/// use unitree_lidar_l1_rust::{Config, Event, Lidar, LidarGroup};
///
/// let lidars = LidarGroup::new()
///     .with("front", Lidar::open(Config::serial("/dev/ttyUSB0"))?)
///     .with("rear", Lidar::open(Config::serial("/dev/ttyUSB1"))?)
///     .spawn();
///
/// for (name, event) in lidars {
///     if let Event::Cloud(cloud) = event {
///         println!("{name}: {} points", cloud.len());
///     }
/// }
/// # Ok::<(), unitree_lidar_l1_rust::Error>(())
/// ```
pub struct LidarGroup<K> {
    lidars: Vec<(Lidar, K)>,
}

impl<K: Clone + Send + 'static> LidarGroup<K> {
    pub fn new() -> Self {
        Self { lidars: Vec::new() }
    }

    pub fn with(mut self, key: K, lidar: Lidar) -> Self {
        self.add(key, lidar);
        self
    }

    pub fn add(&mut self, key: K, lidar: Lidar) -> &mut Self {
        self.lidars.push((lidar, key));
        self
    }

    pub fn len(&self) -> usize {
        self.lidars.len()
    }

    pub fn is_empty(&self) -> bool {
        self.lidars.is_empty()
    }

    /// Starts one background thread per lidar, all feeding a single handle.
    pub fn spawn(self) -> LidarHandle<(K, Event)> {
        LidarHandle::spawn(self.lidars, |key, event| (key, event))
    }

    /// Like [`LidarGroup::spawn`], but the handle can be `.await`ed and is a `Stream`.
    #[cfg(feature = "async")]
    pub fn spawn_async(self) -> AsyncLidarHandle<(K, Event)> {
        AsyncLidarHandle::spawn(self.lidars, |key, event| (key, event))
    }
}

impl<K: Clone + Send + 'static> Default for LidarGroup<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Clone + Send + 'static> FromIterator<(K, Lidar)> for LidarGroup<K> {
    fn from_iter<I: IntoIterator<Item = (K, Lidar)>>(iter: I) -> Self {
        Self {
            lidars: iter.into_iter().map(|(key, lidar)| (lidar, key)).collect(),
        }
    }
}
