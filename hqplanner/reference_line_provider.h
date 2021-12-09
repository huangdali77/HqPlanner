#ifndef HQPLANNER_REFERENCE_LINE_PROVIDER_H_
#define HQPLANNER_REFERENCE_LINE_PROVIDER_H_
#include <list>

#include "reference_line.h"
namespace hqplanner {
class ReferenceLineProvider {
 public:
  bool AddReferenceLine(ReferenceLine& ref_line) {
    reference_lines_.emplace_back(ref_line);
  }

 private:
  std ::list<ReferenceLine> reference_lines_;
};
}  // namespace hqplanner

#endif