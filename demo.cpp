#include <iostream>

#include "process.hpp"

void foo()
{
  std::cout << "foo() called in process " << this_process::get_id() << std::endl;
}

int main()
{
  std::cout << "main() called in process " << this_process::get_id() << std::endl;

  // call foo() in a newly-created process
  process child(foo);

  // wait for the child to complete
  child.join();

  std::cout << "OK" << std::endl;
}

