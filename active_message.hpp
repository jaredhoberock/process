// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

#include "serialization.hpp"
#include "tuple.hpp"


class active_message
{
  public:
    active_message() = default;

    template<class Function, class... Args,
             __REQUIRES(can_serialize_all<Function,Args...>::value),
             __REQUIRES(can_deserialize_all<Function,Args...>::value),
             __REQUIRES(is_invocable<Function,Args...>::value)
            >
    explicit active_message(Function func, Args... args)
      : message_(func, args...)
    {}

    any activate() const
    {
      return message_();
    }

    template<class OutputArchive>
    friend void serialize(OutputArchive& ar, const active_message& self)
    {
      ar(self.message_);
    }

    template<class InputArchive>
    friend void deserialize(InputArchive& ar, active_message& self)
    {
      ar(self.message_);
    }

  private:
    serializable_closure message_;
};


class two_sided_active_message : private active_message
{
  private:
    using super_t = active_message;

    // two_sided_active_message's constructor initializes the base active_message class with this function
    // as the function to call. The user's functions and arguments passed to two_sided_active_message's constructor are this function's arguments.
    // the result of this function is an active_message containing the reply
    template<class Function1, class Tuple1,
             class Function2, class Tuple2>
    static active_message apply_and_return_active_message_reply(Function1 func,       const Tuple1& args1,
                                                                Function2 reply_func, const Tuple2& args2)
    {
      // XXX need to handle the case where user_result is void

      // apply the user's function to the first tuple
      auto user_result = apply(func, args1);

      // concatenate reply_func, user_result, and args2 into a single tuple
      auto constructor_args = std::tuple_cat(std::make_tuple(reply_func, user_result), args2);

      // make an active_message containing the reply
      return make_from_tuple<active_message>(constructor_args);
    }


  public:
    two_sided_active_message() = default;

    // XXX we also need to require that the result of Function1 is serializable/deserializable
    template<class Function1, class Tuple1,
             class Function2, class... Args2,

             // all arguments must be serializable and deserializable
             __REQUIRES(can_serialize_all<Function1,Tuple1,Function2,Args2...>::value),
             __REQUIRES(can_deserialize_all<Function1,Tuple1,Function2,Args2...>::value),

             // auto func_result = func(args1...) must be well-formed
             __REQUIRES(can_apply<Function1,Tuple1>::value),

             class Result1 = apply_result_t<Function1,Tuple1>,

             // reply_func(func_result, args2...) must be well-formed
             // XXX need to handle the case where Result1 is void
             __REQUIRES(is_invocable<Function2,Result1,Args2...>::value)
            >
    two_sided_active_message(Function1 func,       const Tuple1& args1,
                             Function2 reply_func, const std::tuple<Args2...>& args2)
      : super_t(&apply_and_return_active_message_reply<Function1,Tuple1,Function2,std::tuple<Args2...>>,
                func, args1, reply_func, args2)
    {}

    active_message activate() const
    {
      return any_cast<active_message>(super_t::activate());
    }

    template<class OutputArchive>
    friend void serialize(OutputArchive& ar, const two_sided_active_message& self)
    {
      ar(static_cast<const active_message&>(self));
    }

    template<class InputArchive>
    friend void deserialize(InputArchive& ar, two_sided_active_message& self)
    {
      ar(static_cast<active_message&>(self));
    }
};

