// Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
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

#include <type_traits>
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <string>
#include <cstdio>

#include "integer_sequence.hpp"


#define __VARIANT_CONCATENATE_IMPL(x, y) x##y

#define __VARIANT_CONCATENATE(x, y) __VARIANT_CONCATENATE_IMPL(x, y)

#define __VARIANT_MAKE_UNIQUE(x) __VARIANT_CONCATENATE(x, __COUNTER__)

#define __VARIANT_REQUIRES_IMPL(unique_name, ...) bool unique_name = true, typename std::enable_if<(unique_name and __VA_ARGS__)>::type* = nullptr

#define __VARIANT_REQUIRES(...) __VARIANT_REQUIRES_IMPL(__VARIANT_MAKE_UNIQUE(__deduced_true), __VA_ARGS__)


namespace detail
{
namespace variant_detail
{


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


template<size_t i, size_t... js>
struct constexpr_max
{
  static const size_t value = i < constexpr_max<js...>::value ? constexpr_max<js...>::value : i;
};


template<size_t i>
struct constexpr_max<i>
{
  static const size_t value = i;
};


} // end variant_detail
} // end detail


template<typename... Types>
class variant;


template<size_t i, typename Variant> struct variant_alternative;


template<size_t i, typename T0, typename... Types>
struct variant_alternative<i, variant<T0, Types...>>
  : variant_alternative<i-1,variant<Types...>>
{};


template<typename T0, typename... Types>
struct variant_alternative<0, variant<T0, Types...>>
{
  typedef T0 type;
};


template<size_t i, typename... Types>
using variant_alternative_t = typename variant_alternative<i,Types...>::type;


static constexpr const size_t variant_npos = static_cast<size_t>(-1);


namespace detail
{
namespace variant_detail
{


template<typename T, typename U>
struct propagate_reference;


template<typename T, typename U>
struct propagate_reference<T&, U>
{
  typedef U& type;
};


template<typename T, typename U>
struct propagate_reference<const T&, U>
{
  typedef const U& type;
};


template<typename T, typename U>
struct propagate_reference<T&&, U>
{
  typedef U&& type;
};


template<size_t i, typename VariantReference>
struct variant_alternative_reference
  : propagate_reference<
      VariantReference,
      variant_alternative_t<
        i,
        typename std::decay<VariantReference>::type
      >
    >
{};


template<size_t i, typename VariantReference>
using variant_alternative_reference_t = typename variant_alternative_reference<i,VariantReference>::type;


} // end variant_detail
} // end detail


template<typename Visitor, typename Variant>
typename std::result_of<
  Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant&&>)
>::type
  visit(Visitor& visitor, Variant&& var);


template<typename Visitor, typename Variant>
typename std::result_of<
  const Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant&&>)
>::type
  visit(const Visitor& visitor, Variant&& var);


template<typename Visitor, typename Variant1, typename Variant2>
typename std::result_of<
  Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant1&&>,
           detail::variant_detail::variant_alternative_reference_t<0,Variant2&&>)
>::type
  visit(Visitor& visitor, Variant1&& var1, Variant2&& var2);


template<typename Visitor, typename Variant1, typename Variant2>
typename std::result_of<
  const Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant1&&>,
                 detail::variant_detail::variant_alternative_reference_t<0,Variant2&&>)
>::type
  visit(const Visitor& visitor, Variant1&& var1, Variant2&& var2);


namespace detail
{
namespace variant_detail
{


template<size_t i, typename T, typename... Types>
struct find_type_impl;


// no match, keep going
template<size_t i, typename T, typename U, typename... Types>
struct find_type_impl<i,T,U,Types...>
  : find_type_impl<i+1,T,Types...>
{};


// found a match
template<size_t i, typename T, typename... Types>
struct find_type_impl<i,T,T,Types...>
{
  static constexpr const size_t value = i;
};


// no match
template<size_t i, typename T>
struct find_type_impl<i,T>
{
  static constexpr const size_t value = variant_npos;
};


template<typename T, typename... Types>
using find_type = find_type_impl<0,T,Types...>;


template<typename T, typename Variant>
struct is_variant_alternative;

template<class T, class... Types>
struct is_variant_alternative<T,variant<Types...>>
  : std::integral_constant<
      bool,
      (find_type<T,Types...>::value != variant_npos)
    >
{};



// counts the number of occurrences of T in the list Types...
template<class T, class... Types>
struct count_occurrences;

// empty list of types, no occurrences
template<class T>
struct count_occurrences<T>
{
  static const std::size_t value = 0;
};

// no match (T does not match U), continue counting through the remaining Types...
template<class T, class U, class... Types>
struct count_occurrences<T,U,Types...>
{
  static const std::size_t value = count_occurrences<T,Types...>::value;
};

// found a match, continue counting through the remaining Types...
template<class T, class... Types>
struct count_occurrences<T,T,Types...>
{
  static const std::size_t value = 1 + count_occurrences<T,Types...>::value;
};


template<class T, class... Types>
struct occurs_exactly_once : std::integral_constant<bool, count_occurrences<T,Types...>::value == 1> {};



template<size_t i, class... Types>
struct find_exactly_one_impl;

template<size_t i, class T, class U, class... Types>
struct find_exactly_one_impl<i,T,U,Types...> : find_exactly_one_impl<i+1,T,Types...> {};

template<size_t i, class T, class... Types>
struct find_exactly_one_impl<i,T,T,Types...> : std::integral_constant<size_t, i>
{
  static_assert(find_exactly_one_impl<i,T,Types...>::value == variant_npos, "type can only occur once in type list");
};

template<size_t i, class T>
struct find_exactly_one_impl<i,T> : std::integral_constant<size_t, variant_npos> {};

template<class T, class... Types>
struct find_exactly_one : find_exactly_one_impl<0,T,Types...>
{
  static_assert(find_exactly_one::value != variant_npos, "type not found in type list");
};


} // end variant_detail
} // end detail


class bad_variant_access : public std::logic_error
{
  public:
    explicit bad_variant_access(const std::string& what_arg) : logic_error(what_arg) {}
    explicit bad_variant_access(const char* what_arg) : logic_error(what_arg) {}
};


namespace detail
{
namespace variant_detail
{


inline void throw_bad_variant_access(const char* what_arg)
{
  throw bad_variant_access(what_arg);
}


template<class T, class...>
struct first_type
{
  using type = T;
};

template<class... Types>
using first_type_t = typename first_type<Types...>::type;


template<class... Types>
class variant_storage
{
  typedef typename std::aligned_storage<
    constexpr_max<sizeof(Types)...>::value
  >::type storage_type;
  
  storage_type storage_;
};

template<>
class variant_storage<> {};

} // end variant_detail
} // end detail


template<class T>
struct in_place_type_t {};

template<std::size_t I>
struct in_place_index_t {};


template<typename... Types>
class variant : private detail::variant_detail::variant_storage<Types...>
{
  public:
    variant()
      : variant(in_place_index_t<0>())
    {}

  private:
    struct binary_move_construct_visitor
    {
      template<class T>
      void operator()(T& self, T& other) const
      {
        new (&self) T(std::move(other));
      }

      template<class... Args>
      void operator()(Args&&...) const {}
    };

  public:
    template<__VARIANT_REQUIRES(
               detail::variant_detail::conjunction<
                 std::is_move_constructible<Types>...
               >::value
             )>
    variant(variant&& other)
      : index_(other.index())
    {
      auto visitor = binary_move_construct_visitor();
      visit(visitor, *this, other);
    }

  private:
    struct binary_copy_construct_visitor
    {
      template<class T>
      void operator()(T& self, const T& other) const
      {
        new (&self) T(other);
      }

      template<class... Args>
      void operator()(Args&&...) const {}
    };

  public:
    template<__VARIANT_REQUIRES(
               detail::variant_detail::conjunction<
                 std::is_copy_constructible<Types>...
               >::value
             )>
    variant(const variant& other)
      : index_(other.index())
    {
      auto visitor = binary_copy_construct_visitor();
      visit(visitor, *this, other);
    }

    // converting copy constructor
    // XXX we're supposed to select which alternative to use based on some somewhat complex rules that are not yet implemented
    //     instead, just select the first alternative matching T
    template<class T,
             __VARIANT_REQUIRES(
               detail::variant_detail::is_variant_alternative<T,variant>::value
             )>
    variant(const T& other)
      : variant(in_place_index_t<detail::variant_detail::find_type<T,Types...>::value>(), other)
    {}

    // converting move constructor
    // XXX we're supposed to select which alternative to use based on some somewhat complex rules that are not yet implemented
    //     instead, just select the first alternative matching T
    template<class T,
             __VARIANT_REQUIRES(
               detail::variant_detail::is_variant_alternative<typename std::decay<T>::type,variant>::value
             )>
    variant(T&& other)
      : variant(in_place_index_t<detail::variant_detail::find_type<typename std::decay<T>::type,Types...>::value>(), std::forward<T>(other))
    {}

    template<class T, class... Args,
             __VARIANT_REQUIRES(
               detail::variant_detail::occurs_exactly_once<T,Types...>::value &&
               std::is_constructible<T, Args...>::value
             )>
    variant(in_place_type_t<T>, Args&&... args)
      : variant(in_place_index_t<detail::variant_detail::find_exactly_one<T,Types...>::value>(),
                std::forward<Args>(args)...)
    {}

    template<std::size_t I, class... Args,
             __VARIANT_REQUIRES(
               I < sizeof...(Types) and
               std::is_constructible<variant_alternative_t<I,variant>, Args...>::value
             )>
    variant(in_place_index_t<I>, Args&&... args)
      : index_(variant_npos)
    {
      emplace<I>(std::forward<Args>(args)...);
    }

  private:
    struct destruct_visitor
    {
      template<typename T>
      typename std::enable_if<
        !std::is_trivially_destructible<T>::value
      >::type
        operator()(T& x) const
      {
        x.~T();
      }
      
      template<typename T>
      typename std::enable_if<
        std::is_trivially_destructible<T>::value
      >::type
        operator()(T&) const
      {
        // omit invocations of destructors for trivially destructible types
      }
    };

  public:
    ~variant()
    {
      if(!valueless_by_exception())
      {
        auto visitor = destruct_visitor();
        visit(visitor, *this);
      }
    }

  private:
    struct copy_assign_visitor
    {
      template<class T>
      void operator()(T& self, const T& other) const
      {
        self = other;
      }

      template<class... Args>
      void operator()(Args&&...) const {}
    };

    struct destroy_and_copy_construct_visitor
    {
      template<class A, class B>
      void operator()(A& a, const B& b) const
      {
        // copy b to a temporary
        B tmp = b;

        // destroy a
        a.~A();

        // placement move from tmp to a
        new (&a) B(std::move(tmp));
      }
    };

    struct destroy_and_move_construct_visitor
    {
      template<class A, class B>
      void operator()(A& a, B&& b) const
      {
        // destroy a
        a.~A();

        using type = typename std::decay<B>::type;

        // placement move from b
        new (&a) type(std::move(b));
      }
    };

  public:
    template<__VARIANT_REQUIRES(
               detail::variant_detail::conjunction<
                 std::is_copy_assignable<Types>...
               >::value
            )>
    variant& operator=(const variant& other)
    {
      if(index() == other.index())
      {
        visit(copy_assign_visitor(), *this, other);
      }
      else
      {
        visit(destroy_and_copy_construct_visitor(), *this, other);
        index_ = other.index();
      }

      return *this;
    }

  private:
    struct move_assign_visitor
    {
      template<class T>
      void operator()(T& self, T& other) const
      {
        self = std::move(other);
      }

      template<class... Args>
      void operator()(Args&&...) const {}
    };


  public:
    template<__VARIANT_REQUIRES(
               detail::variant_detail::conjunction<
                 std::is_move_assignable<Types>...
               >::value
            )>
    variant& operator=(variant&& other)
    {
      if(index() == other.index())
      {
        auto visitor = move_assign_visitor();
        visit(visitor, *this, other);
      }
      else
      {
        auto visitor = destroy_and_move_construct_visitor();
        visit(visitor, *this, std::move(other));
        index_ = other.index();
      }

      return *this;
    }

    size_t index() const
    {
      return index_;
    }

    bool valueless_by_exception() const
    {
      return index() == variant_npos;
    }

    template<std::size_t I, class... Args,
             __VARIANT_REQUIRES(
               I < sizeof...(Types) and
               std::is_constructible<variant_alternative_t<I,variant>, Args...>::value
             )>
    variant_alternative_t<I, variant>& emplace(Args&&... args)
    {
      // destroy the contained value if it exists
      this->~variant();

      using value_type = variant_alternative_t<I,variant>;

      // placement new into this
      ::new(this) value_type(std::forward<Args>(args)...);

      // set the new index
      index_ = I;

      return *reinterpret_cast<value_type*>(this);
    }

    template<class T, class... Args,
             __VARIANT_REQUIRES(
               detail::variant_detail::occurs_exactly_once<T,Types...>::value &&
               std::is_constructible<T, Args...>::value
             )>
    T& emplace(Args&&... args)
    {
      return emplace<detail::variant_detail::find_exactly_one<T,Types...>::value>(std::forward<Args>(args)...);
    }

  private:
    struct swap_visitor
    {
      template<class A, class B>
      void operator()(A& a, B& b) const
      {
        std::swap(a,b);
      }
    };

  public:
    void swap(variant& other)
    {
      if(index() == other.index())
      {
        auto visitor = swap_visitor();
        visit(visitor, *this, other);
      }
      else
      {
        variant tmp = *this;
        *this = other;
        other = std::move(tmp);
      }
    }

  private:
    struct equals
    {
      template<typename U1, typename U2>
      bool operator()(const U1&, const U2&) const
      {
        return false;
      }

      template<typename T>
      bool operator()(const T& lhs, const T& rhs) const
      {
        return lhs == rhs;
      }
    };


  public:
    bool operator==(const variant& rhs) const
    {
      auto visitor = equals();
      return index() == rhs.index() && visit(visitor, *this, rhs);
    }

    bool operator!=(const variant& rhs) const
    {
      return !operator==(rhs);
    }

  private:
    struct less
    {
      template<typename U1, typename U2>
      bool operator()(const U1&, const U2&) const
      {
        return false;
      }

      template<typename T>
      bool operator()(const T& lhs, const T& rhs) const
      {
        return lhs < rhs;
      }
    };

  public:
    bool operator<(const variant& rhs) const
    {
      if(index() != rhs.index()) return index() < rhs.index();

      return visit(less(), *this, rhs);
    }

    bool operator<=(const variant& rhs) const
    {
      return !(rhs < *this);
    }

    bool operator>(const variant& rhs) const
    {
      return rhs < *this;
    }

    bool operator>=(const variant& rhs) const
    {
      return !(*this < rhs);
    }

  private:
    size_t index_;
};


namespace detail
{
namespace variant_detail
{


struct ostream_output_visitor
{
  std::ostream &os;

  ostream_output_visitor(std::ostream& os) : os(os) {}

  template<typename T>
  std::ostream& operator()(const T& x)
  {
    return os << x;
  }
};


} // variant_detail
} // end detail


template<typename... Types>
std::ostream &operator<<(std::ostream& os, const variant<Types...>& v)
{
  auto visitor = detail::variant_detail::ostream_output_visitor(os);
  return visit(visitor, v);
}


namespace detail
{
namespace variant_detail
{


template<typename VisitorReference, typename Result, typename T, typename... Types>
struct apply_visitor_impl : apply_visitor_impl<VisitorReference,Result,Types...>
{
  typedef apply_visitor_impl<VisitorReference,Result,Types...> super_t;

  static Result do_it(VisitorReference visitor, void* ptr, size_t index)
  {
    if(index == 0)
    {
      return visitor(*reinterpret_cast<T*>(ptr));
    }

    return super_t::do_it(visitor, ptr, --index);
  }


  static Result do_it(VisitorReference visitor, const void* ptr, size_t index)
  {
    if(index == 0)
    {
      return visitor(*reinterpret_cast<const T*>(ptr));
    }

    return super_t::do_it(visitor, ptr, --index);
  }
};


template<typename VisitorReference, typename Result, typename T>
struct apply_visitor_impl<VisitorReference,Result,T>
{
  static Result do_it(VisitorReference visitor, void* ptr, size_t)
  {
    return visitor(*reinterpret_cast<T*>(ptr));
  }

  static Result do_it(VisitorReference visitor, const void* ptr, size_t)
  {
    return visitor(*reinterpret_cast<const T*>(ptr));
  }
};


template<typename VisitorReference, typename Result, typename Variant>
struct apply_visitor;


template<typename VisitorReference, typename Result, typename... Types>
struct apply_visitor<VisitorReference,Result,variant<Types...>>
  : apply_visitor_impl<VisitorReference,Result,Types...>
{};


} // end variant_detail
} // end detail


template<typename Visitor, typename Variant>
typename std::result_of<
  Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant&&>)
>::type
  visit(Visitor& visitor, Variant&& var)
{
  using result_type = typename std::result_of<
    Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant&&>)
  >::type;

  using impl = detail::variant_detail::apply_visitor<Visitor&,result_type,typename std::decay<Variant>::type>;

  return impl::do_it(visitor, &var, var.index());
}


template<typename Visitor, typename Variant>
typename std::result_of<
  const Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant&&>)
>::type
  visit(const Visitor& visitor, Variant&& var)
{
  using result_type = typename std::result_of<
    const Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant&&>)
  >::type;
 
  using impl = detail::variant_detail::apply_visitor<const Visitor&,result_type,typename std::decay<Variant>::type>;

  return impl::do_it(visitor, &var, var.index());
}


namespace detail
{
namespace variant_detail
{


template<typename VisitorReference, typename Result, typename ElementReference>
struct unary_visitor_binder
{
  VisitorReference visitor;
  ElementReference x;

  unary_visitor_binder(VisitorReference visitor, ElementReference x) : visitor(visitor), x(x) {}

  template<typename T>
  Result operator()(T&& y)
  {
    return visitor(x, std::forward<T>(y));
  }
};


template<class Reference>
struct rvalue_reference_to_lvalue_reference
{
  using type = Reference;
};

template<class T>
struct rvalue_reference_to_lvalue_reference<T&&>
{
  using type = T&;
};


template<typename VisitorReference, typename Result, typename VariantReference>
struct binary_visitor_binder
{
  VisitorReference visitor;
  // since rvalue references can't be members of classes, we transform any
  // VariantReference which is an rvalue reference to an lvalue reference
  // when we use y in operator(), we cast it back to the original reference type
  typename rvalue_reference_to_lvalue_reference<VariantReference>::type y;

  binary_visitor_binder(VisitorReference visitor, VariantReference ref) : visitor(visitor), y(ref) {}

  template<typename T>
  Result operator()(T&& x)
  {
    auto unary_visitor = unary_visitor_binder<VisitorReference, Result, decltype(x)>(visitor, std::forward<T>(x));
    return visit(unary_visitor, std::forward<VariantReference>(y));
  }
};


} // end variant_detail
} // end detail


template<typename Visitor, typename Variant1, typename Variant2>
typename std::result_of<
  Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant1&&>,
           detail::variant_detail::variant_alternative_reference_t<0,Variant2&&>)
>::type
  visit(Visitor& visitor, Variant1&& var1, Variant2&& var2)
{
  using result_type = typename std::result_of<
    Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant1&&>,
             detail::variant_detail::variant_alternative_reference_t<0,Variant2&&>)
  >::type;

  auto visitor_wrapper = detail::variant_detail::binary_visitor_binder<Visitor&,result_type,decltype(var2)>(visitor, std::forward<Variant2>(var2));

  return visit(visitor_wrapper, std::forward<Variant1>(var1));
}


template<typename Visitor, typename Variant1, typename Variant2>
typename std::result_of<
  const Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant1&&>,
                 detail::variant_detail::variant_alternative_reference_t<0,Variant2&&>)
>::type
  visit(const Visitor& visitor, Variant1&& var1, Variant2&& var2)
{
  using result_type = typename std::result_of<
    const Visitor&(detail::variant_detail::variant_alternative_reference_t<0,Variant1&&>,
                   detail::variant_detail::variant_alternative_reference_t<0,Variant2&&>)
  >::type;

  auto visitor_wrapper = detail::variant_detail::binary_visitor_binder<const Visitor&,result_type,decltype(var2)>(visitor, std::forward<Variant2>(var2));

  return visit(visitor_wrapper, std::forward<Variant1>(var1));
}


namespace detail
{
namespace variant_detail
{


template<class T>
struct get_visitor
{
  T* operator()(T& x) const
  {
    return &x;
  }

  const T* operator()(const T& x) const
  {
    return &x;
  }

  template<class U>
  T* operator()(U&&) const
  {
    return nullptr;
  }
};


} // end variant_detail
} // end detail


template<size_t i, class... Types>
variant_alternative_t<i, variant<Types...>>&
  get(variant<Types...>& v)
{
  if(i != v.index())
  {
    detail::variant_detail::throw_bad_variant_access("i does not equal index()");
  }

  using type = typename std::decay<
    variant_alternative_t<i,variant<Types...>>
  >::type;

  auto visitor = detail::variant_detail::get_visitor<type>();
  return *visit(visitor, v);
}


template<size_t i, class... Types>
variant_alternative_t<i, variant<Types...>>&&
  get(variant<Types...>&& v)
{
  if(i != v.index())
  {
    detail::variant_detail::throw_bad_variant_access("i does not equal index()");
  }

  using type = typename std::decay<
    variant_alternative_t<i,variant<Types...>>
  >::type;

  auto visitor = detail::variant_detail::get_visitor<type>();
  return std::move(*visit(visitor, v));
}


template<size_t i, class... Types>
const variant_alternative_t<i, variant<Types...>>&
  get(const variant<Types...>& v)
{
  if(i != v.index())
  {
    detail::variant_detail::throw_bad_variant_access("i does not equal index()");
  }

  using type = typename std::decay<
    variant_alternative_t<i,variant<Types...>>
  >::type;

  auto visitor = detail::variant_detail::get_visitor<type>();
  return *visit(visitor, v);
}


template<class T, class... Types>
bool holds_alternative(const variant<Types...>& v)
{
  constexpr size_t i = detail::variant_detail::find_exactly_one<T,Types...>::value;
  return i == v.index();
}


template<class T, class... Types>
typename std::remove_reference<T>::type&
  get(variant<Types...>& v)
{
  return get<detail::variant_detail::find_type<T,Types...>::value>(v);
}


template<class T, class... Types>
const typename std::remove_reference<T>::type&
  get(const variant<Types...>& v)
{
  return get<detail::variant_detail::find_type<T,Types...>::value>(v);
}


template<class T, class... Types>
typename std::remove_reference<T>::type&&
  get(variant<Types...>&& v)
{
  return std::move(get<detail::variant_detail::find_type<T,Types...>::value>(v));
}


struct monostate {};

