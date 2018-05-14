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

#include "new_posix_process_executor.hpp"
#include <sys/wait.h>
#include <utility>


class process
{
  public:
    using id = pid_t;

    // consider adding an overload which takes an executor which creates new processes
    template<class Function>
    process(Function&& f)
      : id_(create_process_with_executor(std::forward<Function>(f)))
    {}

    inline ~process() noexcept
    {
      if(joinable())
      {
        join();
      }
    }

    inline id get_id() const noexcept
    {
      return id_;
    }

    inline bool joinable() const noexcept
    {
      return get_id() >= 0;
    }

    inline void join() noexcept
    {
      waitpid(get_id(), nullptr, 0);
      detach();
    }

    inline void detach() noexcept
    {
      id_ = -1;
    }

    inline void swap(process& other) noexcept
    {
      std::swap(id_, other.id_);
    }

  private:
    template<class Function>
    static id create_process_with_executor(Function&& f)
    {
      new_posix_process_executor ex;
      ex.execute(std::forward<Function>(f));
      return ex.query(last_created_process_id);
    }

    id id_;
};

