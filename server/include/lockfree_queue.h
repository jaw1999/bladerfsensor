#ifndef LOCKFREE_QUEUE_H
#define LOCKFREE_QUEUE_H

#include <atomic>
#include <memory>
#include <cstring>
#include <vector>
#include <fftw3.h>

// Lock-free Single Producer Single Consumer (SPSC) ring buffer
// Optimized for high-throughput, low-latency data transfer between threads
// Thread-safe for exactly one producer and one consumer thread
template<typename T>
class LockFreeQueue {
public:
    explicit LockFreeQueue(size_t capacity)
        : capacity_(capacity + 1),  // +1 to distinguish full from empty
          buffer_(new T[capacity + 1]),
          head_(0),
          tail_(0) {
    }

    ~LockFreeQueue() {
        delete[] buffer_;
    }

    // Producer: Push item onto queue (returns false if full)
    bool push(const T& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = increment(current_tail);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false;  // Queue is full
        }

        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    // Consumer: Pop item from queue (returns false if empty)
    bool pop(T& item) {
        const size_t current_head = head_.load(std::memory_order_relaxed);

        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false;  // Queue is empty
        }

        item = buffer_[current_head];
        head_.store(increment(current_head), std::memory_order_release);
        return true;
    }

    // Check if queue is empty (may be stale immediately after call)
    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    // Check if queue is full (may be stale immediately after call)
    bool full() const {
        const size_t next_tail = increment(tail_.load(std::memory_order_acquire));
        return next_tail == head_.load(std::memory_order_acquire);
    }

    // Get approximate size (may be stale)
    size_t size() const {
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_acquire);
        if (tail >= head) {
            return tail - head;
        } else {
            return capacity_ - head + tail;
        }
    }

    // Get capacity
    size_t capacity() const {
        return capacity_ - 1;
    }

private:
    size_t increment(size_t idx) const {
        return (idx + 1) % capacity_;
    }

    const size_t capacity_;
    T* buffer_;

    // Align to cache line to prevent false sharing between producer/consumer
    alignas(64) std::atomic<size_t> head_;  // Consumer index
    alignas(64) std::atomic<size_t> tail_;  // Producer index

    // Prevent copying
    LockFreeQueue(const LockFreeQueue&) = delete;
    LockFreeQueue& operator=(const LockFreeQueue&) = delete;
};

// Data structure for passing samples between pipeline stages
struct SampleBuffer {
    std::vector<int16_t> samples;  // Interleaved IQ samples (4 channels)
    size_t count;                   // Number of IQ pairs (not total int16_t count)
    uint64_t timestamp_us;          // Timestamp when samples were acquired

    SampleBuffer() : count(0), timestamp_us(0) {}

    explicit SampleBuffer(size_t size) : samples(size), count(0), timestamp_us(0) {}
};

// Wrapper for fftwf_complex to make it copyable in vectors
struct ComplexSample {
    float real;
    float imag;

    ComplexSample() : real(0.0f), imag(0.0f) {}
    ComplexSample(float r, float i) : real(r), imag(i) {}

    // Convert from fftwf_complex
    void from_fftw(const fftwf_complex& c) {
        real = c[0];
        imag = c[1];
    }

    // Convert to fftwf_complex
    void to_fftw(fftwf_complex& c) const {
        c[0] = real;
        c[1] = imag;
    }
};

// Data structure for passing FFT results between pipeline stages
struct FFTBuffer {
    std::vector<uint8_t> ch1_mag;           // Channel 1 magnitude (0-255)
    std::vector<uint8_t> ch2_mag;           // Channel 2 magnitude (0-255)
    std::vector<ComplexSample> ch1_fft;     // Channel 1 complex FFT
    std::vector<ComplexSample> ch2_fft;     // Channel 2 complex FFT
    size_t size;                            // FFT size
    uint64_t timestamp_us;                  // Processing timestamp
    float noise_floor_ch1;                  // Noise floor estimate CH1
    float noise_floor_ch2;                  // Noise floor estimate CH2

    FFTBuffer() : size(0), timestamp_us(0), noise_floor_ch1(0.0f), noise_floor_ch2(0.0f) {}

    explicit FFTBuffer(size_t fft_size)
        : ch1_mag(fft_size), ch2_mag(fft_size),
          ch1_fft(fft_size), ch2_fft(fft_size),
          size(fft_size), timestamp_us(0), noise_floor_ch1(0.0f), noise_floor_ch2(0.0f) {}
};

#endif // LOCKFREE_QUEUE_H
