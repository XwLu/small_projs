#include "osqp_problem.h"
#include <math.h>
#include <fstream>
#include <sstream>
using namespace std;

using PathBoundary = std::vector<std::pair<double, double>>;

class PathBoundaryInfo
{
public:
  inline void set_start_s(double start_s) { start_s_ = start_s; }
  inline void set_delta_s(double delta_s) { delta_s_ = delta_s; }
  inline void set_boundary(const PathBoundary &boundary) { boundary_ = boundary; }

  inline double delta_s() { return delta_s_; }
  inline const PathBoundary &boundary() { return boundary_; }
  PathBoundary boundary_;

private:
  double start_s_;
  double delta_s_;
};

void build_path_boundary(PathBoundaryInfo &bound_info, double planning_length,
                         double delta_s, double start_s)
{
  bound_info.set_start_s(start_s);
  bound_info.set_delta_s(delta_s);

  int num = floor(planning_length / delta_s);
  PathBoundary bound;
  for (int i = 0; i < num; i++)
  {
    bound.emplace_back(std::make_pair(-3.0, 3.0));
  }

  int start = floor(num / 3.0);
  int end = floor(num / 2.0);

  for (int i = 0; i < start; i++)
  {
    bound[i].first = -0.5;
  }

  for (int i = start; i < end; i++)
  {
    bound[i].first = -2.0;
    bound[i].second = 1.0;
  }

  for (int i = end; i < num - 20; i++)
  {
    bound[i].first = 0.0;
    bound[i].second = 2.5;
  }

  for (int i = num - 20; i < num; i++)
  {
    bound[i].first = -2.5;
    bound[i].second = 1.5;
  }

  bound_info.set_boundary(bound);
}

int main(int argc, char **argv)
{
  std::cout << "OSQP RUNNING..." << std::endl;
  PathBoundaryInfo bound_info;
  build_path_boundary(bound_info, 50.0, 0.5, 0.0);

  std::array<double, 3> init_state = {2.0, 0.0, 0.0};
  OSQPProblem prob(bound_info.boundary().size(), bound_info.delta_s(),
                   init_state);

  std::array<double, 3> end_state = {-1.5, 0.0, 0.0};
  prob.set_end_state_ref({1000.0, 10.0, 10.0}, end_state);

  std::vector<double> x_ref(bound_info.boundary().size(), end_state[0]);

  // prob.set_x_ref(10.0, x_ref);

  prob.set_weight_x(4.0);
  prob.set_weight_dx(20.0);
  prob.set_weight_ddx(1000.0);
  prob.set_weight_dddx(0.0);

  prob.set_scale_factor({1.0, 10.0, 100.0});

  prob.set_x_bounds(bound_info.boundary());
  prob.set_dx_bounds(-2, 2);
  prob.set_ddx_bounds(-0.2, 0.2);
  prob.set_dddx_bounds(-0.1, 0.1);

  ofstream mycout;

  time_t nowtime = time(NULL);
  struct tm *p;
  p = gmtime(&nowtime);
  char filename[256] = {0};
  char timeinfo[256] = {0};
  sprintf(timeinfo, "%d-%d-%d %d-%02d-osqp-traj", 1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday, 8 + p->tm_hour, p->tm_min);
  mycout.open(timeinfo);

  if (prob.Optimize(1000))
  {
    std::cout << "Optimize successful!!" << std::endl;
    for (int i = 0; i < prob.x_.size(); ++i)
    {
      mycout << "x: " << prob.x_.at(i) << " dx: " << prob.dx_.at(i) << " ddx: " << prob.ddx_.at(i) << " left_edge: " << bound_info.boundary_.at(i).second << " right_edge: " << bound_info.boundary_.at(i).first << std::endl;
    }
  }
  else
  {
    std::cout << "Optimize failed!!" << std::endl;
  }
  // std::cout<<"OSQP END!!!"<<std::endl;
  mycout.close();
  return 0;
};
