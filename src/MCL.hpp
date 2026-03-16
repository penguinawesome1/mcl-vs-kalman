#include <stdexcept>

namespace MCL {

class Filter {
 public:
  //   float example() { throw std::logic_error("Not implemented"); }

 private:
  struct Particle {
    double x, y, theta;
    double weight;
  };
};

}  // namespace MCL