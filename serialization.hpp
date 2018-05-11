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

#include <iostream>
#include <tuple>
#include <string>
#include <typeinfo>
#include <sstream>
#include <cstring>
#include "string_view_stream.hpp"
#include "tuple.hpp"
#include "variant.hpp"


#define __REQUIRES(...) typename std::enable_if<(__VA_ARGS__)>::type* = nullptr


// this serialization scheme is based on Cereal
// see http://uscilab.github.io/cereal

template<class OutputArchive, class T>
void serialize(OutputArchive& ar, const T& value)
{
  // by default, use formatted output, and follow with whitespace
  ar.stream() << value << " ";
}

template<class OutputArchive, class Result, class... Args>
void serialize(OutputArchive& ar, Result (*const &fun_ptr)(Args...))
{
  void* void_ptr = reinterpret_cast<void*>(fun_ptr);

  serialize(ar, void_ptr);
}

template<class OutputArchive>
void serialize(OutputArchive& ar, const std::string& s)
{
  // output the length
  serialize(ar, s.size());

  // output the bytes
  ar.stream().write(s.data(), s.size());
}


template<class InputArchive, class T>
void deserialize(InputArchive& ar, T& value)
{
  // by default, use formatted input, and consume trailing whitespace
  ar.stream() >> value >> std::ws;
}

template<class InputArchive, class T,
         __REQUIRES(!std::is_void<T>::value)>
void deserialize(InputArchive& ar, T*& ptr)
{
  void* void_ptr = nullptr;
  deserialize(ar, void_ptr);

  ptr = reinterpret_cast<T*>(void_ptr);
}

template<class InputArchive, class Result, class... Args>
void deserialize(InputArchive& ar, Result (*&fun_ptr)(Args...))
{
  void* void_ptr = nullptr;
  deserialize(ar, void_ptr);

  using function_ptr_type = Result (*)(Args...);
  fun_ptr = reinterpret_cast<function_ptr_type>(void_ptr);
}


template<class InputArchive>
void deserialize(InputArchive& ar, std::string& s)
{
  // read the length and resize the string
  std::size_t length = 0;
  deserialize(ar, length);
  s.resize(length);

  // read characters from the stream
  ar.stream().read(&s.front(), length);
}


template<size_t Index, class OutputArchive, class... Ts, __REQUIRES(Index == sizeof...(Ts))>
void serialize_tuple_impl(OutputArchive& ar, const std::tuple<Ts...>& tuple)
{
}

template<size_t Index, class OutputArchive, class... Ts, __REQUIRES(Index < sizeof...(Ts))>
void serialize_tuple_impl(OutputArchive& ar, const std::tuple<Ts...>& tuple)
{
  serialize(ar, std::get<Index>(tuple));
  serialize_tuple_impl<Index+1>(ar, tuple);
}

template<class OutputArchive, class... Ts>
void serialize(OutputArchive& ar, const std::tuple<Ts...>& tuple)
{
  serialize_tuple_impl<0>(ar, tuple);
}


template<size_t Index, class InputArchive, class... Ts, __REQUIRES(Index == sizeof...(Ts))>
void deserialize_tuple_impl(InputArchive& ar, std::tuple<Ts...>& tuple)
{
}

template<size_t Index, class InputArchive, class... Ts, __REQUIRES(Index < sizeof...(Ts))>
void deserialize_tuple_impl(InputArchive& ar, std::tuple<Ts...>& tuple)
{
  deserialize(ar, std::get<Index>(tuple));
  deserialize_tuple_impl<Index+1>(ar, tuple);
}

template<class InputArchive, class... Ts>
void deserialize(InputArchive& ar, std::tuple<Ts...>& tuple)
{
  deserialize_tuple_impl<0>(ar, tuple);
}


template<class OutputArchive>
struct serialize_visitor
{
  OutputArchive& ar;

  template<class T>
  void operator()(const T& value) const
  {
    ar(value);
  }
};


template<class OutputArchive, class... Types>
void serialize(OutputArchive& ar, const variant<Types...>& v)
{
  // serialize the index
  ar(v.index());

  // serialize the value
  visit(serialize_visitor<OutputArchive>{ar}, v);
}


template<size_t i, class InputArchive, class... Ts, __REQUIRES(i == sizeof...(Ts))>
void deserialize_variant_impl(InputArchive& ar, size_t index, variant<Ts...>& v)
{
  throw std::runtime_error("deserialize(InputArchive,variant): invalid index.");
}

template<size_t i, class InputArchive, class... Ts, __REQUIRES(i < sizeof...(Ts))>
void deserialize_variant_impl(InputArchive& ar, size_t index, variant<Ts...>& v)
{
  if(index == i)
  {
    variant_alternative_t<i, variant<Ts...>> value;
    ar(value);

    v = value;
  }
  else
  {
    deserialize_variant_impl<i+1>(ar, index, v);
  }
}

template<class InputArchive, class... Types>
void deserialize(InputArchive& ar, variant<Types...>& v)
{
  // deserialize the index
  size_t index;
  ar(index);

  deserialize_variant_impl<0>(ar, index, v);
}


class output_archive
{
  private:
    // this is the terminal case of operator() above
    // it never needs to be called by a client
    inline void operator()() {}

    std::ostream& stream_;

  public:
    inline output_archive(std::ostream& os)
      : stream_(os)
    {}

    inline ~output_archive()
    {
      stream_.flush();
    }

    template<class Arg, class... Args>
    void operator()(const Arg& arg, const Args&... args)
    {
      serialize(*this, arg);

      (*this)(args...);
    }

    inline std::ostream& stream()
    {
      return stream_;
    }
};

class input_archive
{
  public:
    inline input_archive(std::istream& is)
      : stream_(is)
    {}

    template<class Arg, class... Args>
    void operator()(Arg& arg, Args&... args)
    {
      deserialize(*this, arg);

      (*this)(args...);
    }

    inline std::istream& stream()
    {
      return stream_;
    }

  private:
    // this is the terminal case of operator() above
    // it never needs to be called by a client
    inline void operator()() {}

    std::istream& stream_;
};


class any;

template<class ValueType>
ValueType any_cast(const any& self);


class any
{
  public:
    any() = default;

    template<class T>
    any(T&& value)
    {
      std::stringstream os;

      {
        output_archive archive(os);

        archive(value);
      }

      representation_ = os.str();
    }

    template<class ValueType>
    friend ValueType any_cast(const any& self)
    {
      ValueType result;

      std::stringstream is(self.representation_);
      input_archive archive(is);

      archive(result);

      return result;
    }

  private:
    std::string representation_;
};


template<class OutputArchive>
void serialize(OutputArchive& ar, const any& a)
{
  ar.stream() << a;
}

template<class InputArchive>
void deserialize(InputArchive& ar, any& a)
{
  ar.stream() >> a;
}


template<class... Conditions>
struct conjunction;

template<>
struct conjunction<> : std::true_type {};

template<class Condition, class... Conditions>
struct conjunction<Condition, Conditions...>
  : std::integral_constant<
      bool,
      Condition::value && conjunction<Conditions...>::value
    >
{};


template<class T>
struct can_serialize_impl
{
  template<class U,
           class Result = decltype(serialize(std::declval<output_archive&>(), std::declval<U>()))
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<T>(0));
};

template<class T>
using can_serialize = typename can_serialize_impl<T>::type;

template<class... Ts>
using can_serialize_all = conjunction<can_serialize<Ts>...>;


template<class T>
struct can_deserialize_impl
{
  template<class U,
           class Result = decltype(deserialize(std::declval<input_archive&>(), std::declval<U&>()))
          >
  static std::true_type test(int);

  template<class>
  static std::false_type test(...);

  using type = decltype(test<T>(0));
};

template<class T>
using can_deserialize = typename can_deserialize_impl<T>::type;


template<class... Ts>
using can_deserialize_all = conjunction<can_deserialize<Ts>...>;



class serializable_closure
{
  private:
    static void noop_function() {}

  public:
    serializable_closure()
      : serializable_closure(&noop_function)
    {}

    template<class Function, class... Args,
             __REQUIRES(can_serialize_all<Function,Args...>::value),
             __REQUIRES(can_deserialize_all<Function,Args...>::value),
             __REQUIRES(is_invocable<Function,Args...>::value)
            >
    explicit serializable_closure(Function func, Args... args)
      : serialized_(serialize_function_and_arguments(&deserialize_and_invoke<Function,Args...>, func, args...))
    {}

    any operator()() const
    {
      std::stringstream is(serialized_);
      input_archive archive(is);

      // extract a function_ptr_type from the beginning of the buffer
      using function_ptr_type = any (*)(input_archive&);
      function_ptr_type invoke_me = nullptr;
      archive(invoke_me);

      // invoke the function pointer on the remaining data
      return invoke_me(archive);
    }

    template<class OutputArchive>
    friend void serialize(OutputArchive& ar, const serializable_closure& sc)
    {
      ar(sc.serialized_);
    }

    template<class InputArchive>
    friend void deserialize(InputArchive& ar, serializable_closure& sc)
    {
      ar(sc.serialized_);
    }

  private:
    template<class Function, class Tuple,
             class ApplyResult = decltype(apply(std::declval<Function&&>(), std::declval<Tuple&&>())),
             __REQUIRES(std::is_void<ApplyResult>::value)
            >
    static any apply_and_return_any(Function&& f, Tuple&& t)
    {
      apply(std::forward<Function>(f), std::forward<Tuple>(t));
      return any();
    }

    template<class Function, class Tuple,
             class ApplyResult = decltype(apply(std::declval<Function&&>(), std::declval<Tuple&&>())),
             __REQUIRES(!std::is_void<ApplyResult>::value)
            >
    static any apply_and_return_any(Function&& f, Tuple&& t)
    {
      return apply(std::forward<Function>(f), std::forward<Tuple>(t));
    }

    template<class FunctionPtr, class... Args>
    static any deserialize_and_invoke(input_archive& archive)
    {
      // deserialize function pointer and its arguments
      std::tuple<FunctionPtr,Args...> function_and_args;
      archive(function_and_args);

      // split tuple into a function pointer and arguments
      FunctionPtr f = std::get<0>(function_and_args);
      std::tuple<Args...> arguments = tail(function_and_args);

      return apply_and_return_any(f, arguments);
    }

    template<class... Args>
    static std::string serialize_function_and_arguments(const Args&... args)
    {
      std::stringstream os;

      {
        output_archive archive(os);

        // serialize arguments into the archive
        archive(args...);
      }

      return os.str();
    }

    std::string serialized_;
};


template<class T>
std::string to_string(const T& value)
{
  std::stringstream os;
  output_archive ar(os);
  ar(value);
  return os.str();
}


template<class T>
T from_string(const char* string, std::size_t size)
{
  T result;

  string_view_stream is(string, size);
  input_archive ar(is);
  ar(result);

  return result;
}


template<class T>
T from_string(const char* string)
{
  return from_string<T>(string, std::strlen(string));
}

