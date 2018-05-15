#ifndef PATH
#define PATH
#include <vector>


struct node{
   double x;
   double y;
   int row;
   int column;
};

class Path{

 private:
  Path(Path const &) = delete;
  Path(Path &&) = delete;
  Path &operator=(Path const &) = delete;
  Path &operator=(Path &&) = delete;

 public:
  Path() noexcept;
  ~Path() = default;

 public:
  std::vector<node> getNode(double,double,double,double) noexcept;

 private:
  int m_m;
};

#endif
