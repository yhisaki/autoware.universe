#include <Eigen/Dense>

int main()
{
  Eigen::VectorXd a;
  Eigen::VectorXd b;

  a = Eigen::VectorXd::Zero(3);
  b = Eigen::VectorXd::Zero(3);

  auto c = a + b;

  c.x();

  return 0;
}