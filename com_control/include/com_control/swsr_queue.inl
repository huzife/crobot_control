template <typename T>
SWSR_Queue<T>::SWSR_Queue(std::size_t size)
    : size_(size + 1),
      head_(0),
      tail_(0) {
    buf_ = new T[size_];
}

template <typename T>
SWSR_Queue<T>::~SWSR_Queue() {
    if (buf_) {
        delete[] buf_;
    }
}

template <typename T>
bool SWSR_Queue<T>::push(T val) {
    auto cur_tail = tail_.load();
    auto next_tail = next_index(cur_tail);

    // full
    if (next_tail == head_.load())
        return false;

    // push element
    buf_[cur_tail] = val;
    tail_.store(next_tail);

    return true;
}

template <typename T>
bool SWSR_Queue<T>::pop(T& val) {
    auto cur_head = head_.load();

    // empty
    if (cur_head == tail_.load())
        return false;

    // pop element
    val = buf_[cur_head];
    head_.store(next_index(cur_head));

    return true;
}

template <typename T>
size_t SWSR_Queue<T>::next_index(std::size_t index) const {
    return (index + 1) % size_;
}
