
/////////////////////////////////////////////////////////////////
// The start_s and end_s are longitudinal values.
// start_s <= end_s.
//
//              end_s
//                ^
//                |
//          S  direction
//                |
//            start_s
//
// The start_l and end_l are lateral values.
// start_l <= end_l. Left side of the reference line is positive,
// and right side of the reference line is negative.
//  end_l  <-----L direction---- start_l
/////////////////////////////////////////////////////////////////

struct SLBoundary {
  double start_s;
  double end_s;
  double start_l;
  double end_l;
};