#ifndef HQPLANNER_MATH_INDEXED_QUEUE_H_
#define HQPLANNER_MATH_INDEXED_QUEUE_H_

// #include <google/protobuf/stubs/common.h>

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

// #include "google/protobuf/stubs/map_util.h"
// #include "modules/common/util/map_util.h"
namespace hqplanner {
namespace math {

template <typename I, typename T>
class IndexedQueue {
 public:
  // Get infinite capacity with 0.
  explicit IndexedQueue(std::size_t capacity) : capacity_(capacity) {}
  const T *Find(const I id) const {
    // std::unordered_map<I, std::unique_ptr<T>>::iterator it =
    //     map_.find(id);
    map_.find(id);
    if (it == map_.end()) {
      return nullptr;
    }
    return (it->second).get();
    // auto *result = apollo::common::util::FindOrNull(map_, id);
    // return result ? result->get() : nullptr;
  }

  const T *Latest() const {
    if (queue_.empty()) {
      return nullptr;
    }
    return Find(queue_.back().first);
  }

  bool Add(const I id, std::unique_ptr<T> ptr) {
    if (Find(id)) {
      return false;
    }
    if (capacity_ > 0 && queue_.size() == capacity_) {
      map_.erase(queue_.front().first);
      queue_.pop();
    }
    queue_.push(std::make_pair(id, ptr.get()));
    map_[id] = std::move(ptr);
    return true;
  }

  void Clear() {
    capacity_ = 0;
    while (!queue_.empty()) {
      queue_.pop();
    }
    map_.clear();
  }

 public:
  std::size_t capacity_ = 0;
  std::queue<std::pair<I, const T *>> queue_;
  std::unordered_map<I, std::unique_ptr<T>> map_;
};

}  // namespace math
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_INDEXED_QUEUE_H_
