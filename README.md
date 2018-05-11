# process
A `std::thread`-alike for processes.

# Demonstration

To create a new process, use `process` like you would `std::thread`:

```c++
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
```

Program output:

    $ clang -std=c++11 demo.cpp -lstdc++
    $ ./a.out 
    main() called in process 67869
    foo() called in process 67870
    OK

