#ifndef HQPLANNER_FOR_PROTO_DECISION_H_
#define HQPLANNER_FOR_PROTO_DECISION_H_

namespace hqplanner {
namespace forproto {
struct ObjectSidePass {
  enum Type { LEFT = 1, RIGHT = 2 };
  Type type;
};

}  // namespace forproto
}  // namespace hqplanner
