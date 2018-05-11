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

#include <tuple> 
#include <utility>
#include <type_traits>

#include "integer_sequence.hpp"


template<size_t... Indices, class T, class... Ts>
static std::tuple<Ts...> tail_impl(index_sequence<Indices...>, const std::tuple<T,Ts...>& t)
{
  return std::tuple<Ts...>(std::get<1 + Indices>(t)...);
}

template<class T, class... Ts>
static std::tuple<Ts...> tail(const std::tuple<T,Ts...>& t)
{
  return tail_impl(make_index_sequence<sizeof...(Ts)>(), t);
}

template<size_t... Indices, class Function, class Tuple>
static auto apply_impl(index_sequence<Indices...>, Function&& f, Tuple&& t) ->
  decltype(
    std::forward<Function>(f)(std::get<Indices>(std::forward<Tuple>(t))...)
  )
{
  return std::forward<Function>(f)(std::get<Indices>(std::forward<Tuple>(t))...);
}

template<class Function, class Tuple>
static auto apply(Function&& f, Tuple&& t) ->
  decltype(
    apply_impl(
      make_index_sequence<std::tuple_size<typename std::decay<Tuple>::type>::value>(),
      std::forward<Function>(f),
      std::forward<Tuple>(t)
    )
  )
{
  static constexpr size_t num_args = std::tuple_size<typename std::decay<Tuple>::type>::value;
  return apply_impl(make_index_sequence<num_args>(), std::forward<Function>(f), std::forward<Tuple>(t));
}


template<class Function, class... Args>
struct is_invocable_impl
{
  template<class F,
           class Result = decltype(std::declval<F>()(std::declval<Args>()...))>
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<Function>(0));
};

template<class Function, class... Args>
using is_invocable = typename is_invocable_impl<Function,Args...>::type;


template<class Function, class... Args>
struct invoke_result
{
  using type = decltype(std::declval<Function>()(std::declval<Args>()...));
};

template<class Function, class... Args>
using invoke_result_t = typename invoke_result<Function,Args...>::type;


template<class Function, class Tuple>
struct can_apply_impl
{
  template<class F,
           class Result = decltype(apply(std::declval<Function>(), std::declval<Tuple>()))
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<Function>(0));
};

template<class Function, class Tuple>
using can_apply = typename can_apply_impl<Function,Tuple>::type;


template<class Function, class Tuple>
using apply_result_t = decltype(apply(std::declval<Function>(), std::declval<Tuple>()));


template<class T, class Tuple, std::size_t... I>
constexpr T make_from_tuple_impl(Tuple&& t, index_sequence<I...>)
{
  return T(std::get<I>(std::forward<Tuple>(t))...);
}


template<class T, class Tuple>
constexpr T make_from_tuple(Tuple&& t)
{
  return make_from_tuple_impl<T>(std::forward<Tuple>(t), make_index_sequence<std::tuple_size<typename std::decay<Tuple>::type>::value>{});
}

