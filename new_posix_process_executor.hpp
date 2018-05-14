// Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <limits.h>
#include <unistd.h>
#include <spawn.h>
#include <stdexcept>

#include <vector>
#include <utility>
#include <algorithm>

#include "active_message.hpp"


extern char** environ;

namespace this_process
{


static inline pid_t get_id()
{
  return getpid();
}


static inline const std::vector<std::string>& environment()
{
  static std::vector<std::string> result;

  if(result.empty())
  {
    for(char** variable = environ; *variable; ++variable)
    {
      result.push_back(std::string(*variable));
    }
  }

  return result;
}


static inline const std::string& filename()
{
  static std::string result;

  if(result.empty())
  {
    std::string symbolic_name = std::string("/proc/") + std::to_string(getpid()) + "/exe";

    char real_name[PATH_MAX + 1];
    ssize_t length = readlink(symbolic_name.c_str(), real_name, PATH_MAX);
    if(length == -1)
    {
      throw std::runtime_error("this_process::filename(): Error after readlink().");
    }

    real_name[length] = '\0';

    result = real_name;
  }

  return result;
}


namespace detail
{


static inline void set_variable(std::vector<std::string>& environment, const std::string& variable, const std::string& value)
{
  auto existing_variable = std::find_if(environment.begin(), environment.end(), [&](const std::string& current_variable)
  {
    // check if variable is a prefix of current_variable
    auto result = std::mismatch(variable.begin(), variable.end(), current_variable.begin());
    if(result.first == variable.end())
    {
      // check if the next character after the prefix is an equal sign
      return result.second != variable.end() && *result.second == '=';
    }

    return false;
  });

  if(existing_variable != environment.end())
  {
    *existing_variable = variable + "=" + value;
  }
  else
  {
    environment.emplace_back(variable + "=" + value);
  }
}


static inline std::vector<char*> environment_view(const std::vector<std::string>& environment)
{
  std::vector<char*> result;

  for(const std::string& variable : environment)
  {
    result.push_back(const_cast<char*>(variable.c_str()));
  }

  // the view is assumed to be null-terminated
  result.push_back(0);

  return result;
}


// this replaces a process's execution of main() with an active_message if
// the environment variable EXECUTE_ACTIVE_MESSAGE_BEFORE_MAIN is defined
struct maybe_execute_active_message_and_exit
{
  maybe_execute_active_message_and_exit()
  {
    char* variable = std::getenv("EXECUTE_ACTIVE_MESSAGE_BEFORE_MAIN");
    if(variable)
    {
      active_message message = from_string<active_message>(variable);
      message.activate();

      std::exit(EXIT_SUCCESS);
    }
  }
};

maybe_execute_active_message_and_exit alternate_main{};


} // end detail
} // end this_process


struct last_created_process_id_t
{
  static constexpr bool is_requirable = false;
  static constexpr bool is_preferable = false;
};

constexpr last_created_process_id_t last_created_process_id{};


class new_posix_process_executor
{
  public:
    new_posix_process_executor() noexcept
      : last_created_process_id_(-1)
    {}

    pid_t query(last_created_process_id_t) const noexcept
    {
      return last_created_process_id_;
    }

    template<class Function>
    void execute(Function&& f) const
    {
      last_created_process_id_ = create_new_process(active_message(std::forward<Function>(f)));
    }

  private:
    static pid_t create_new_process(active_message message)
    {
      // make a copy of this process's environment and set a variable to contain the serialized active message
      auto child_environment = this_process::environment();
      this_process::detail::set_variable(child_environment, "EXECUTE_ACTIVE_MESSAGE_BEFORE_MAIN", to_string(message));
      auto child_environment_view = this_process::detail::environment_view(child_environment);

      pid_t child_id;
      char* child_argv[] = {nullptr};
      int error = posix_spawn(&child_id, this_process::filename().c_str(), nullptr, nullptr, child_argv, child_environment_view.data());
      if(error)
      {
        throw std::runtime_error("Error after posix_spawn");
      }

      return child_id;
    }

    mutable pid_t last_created_process_id_;
};

