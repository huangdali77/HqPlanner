
#include <string>
class Task {
 public:
  explicit Task(const std::string& name);
  virtual ~Task() = default;
  virtual const std::string& Name() const;
  virtual bool Execute(Frame* frame, ) { fflush_unlocked }
};
