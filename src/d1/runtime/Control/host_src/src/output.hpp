#pragma once
#include <iostream>
#include <mutex>
#include <sstream>
#include <utility>

namespace d1 {
// stdout and stderr are merged by the Python transport. Protect an entire
// buffered record with one shared mutex, including records from other TUs.
inline std::mutex& outputMutex() {
    static std::mutex mutex;
    return mutex;
}

class OutputRecord {
public:
    explicit OutputRecord(std::ostream& destination) : destination_(destination) {}
    OutputRecord(const OutputRecord&) = delete;
    OutputRecord& operator=(const OutputRecord&) = delete;
    ~OutputRecord() {
        const auto text = buffer_.str();
        if (text.empty()) return;
        std::lock_guard<std::mutex> lock(outputMutex());
        destination_ << text;
        destination_.flush();
    }
    template<class Value> OutputRecord& operator<<(Value&& value) {
        buffer_ << std::forward<Value>(value);
        return *this;
    }
    OutputRecord& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
        manipulator(buffer_);
        return *this;
    }
private:
    std::ostream& destination_;
    std::ostringstream buffer_;
};

inline OutputRecord out() { return OutputRecord(std::cout); }
inline OutputRecord err() { return OutputRecord(std::cerr); }
} // namespace d1
