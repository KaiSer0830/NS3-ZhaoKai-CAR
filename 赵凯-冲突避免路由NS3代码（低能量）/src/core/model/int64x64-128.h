/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/core-config.h"

#if !defined(INT64X64_128_H) && defined (INT64X64_USE_128) && !defined(PYTHON_SCAN)
#define INT64X64_128_H

#include <stdint.h>
#include <cmath>  // pow

#if defined(HAVE___UINT128_T) && !defined(HAVE_UINT128_T)
typedef __uint128_t uint128_t;
typedef __int128_t int128_t;
#endif

/**
 * \file
 * \ingroup highprec
 * Declaration of the ns3::int64x64_t type using a native int128_t type..
 */

namespace ns3 {

/**
 * \internal
 * The implementation documented here is based on native 128-bit integers.
 */
class int64x64_t
{
  /// uint128_t high bit (sign bit).
  static const uint128_t   HP128_MASK_HI_BIT = (((int128_t)1)<<127);
  /// Mask for fraction part.
  static const uint64_t    HP_MASK_LO = 0xffffffffffffffffULL;
  /// Mask for sign + integer part.
  static const uint64_t    HP_MASK_HI = ~HP_MASK_LO;
  /**
   * Floating point value of HP_MASK_LO + 1.
   * We really want:
   * \code
   *   static const long double HP_MAX_64 = std:pow (2.0L, 64);
   * \endcode
   * but we can't call functions in const definitions.
   *
   * We could make this a static and initialize in int64x64-128.cc or
   * int64x64.cc, but this requires handling static initialization order
   * when most of the implementation is inline.  Instead, we resort to
   * this define.
   */
#define HP_MAX_64    (std::pow (2.0L, 64))

public:
  /**
   * Type tag for the underlying implementation.
   *
   * A few testcases are are sensitive to implementation,
   * specifically the double implementation.  To handle this,
   * we expose the underlying implementation type here.
   */
  enum impl_type {
    int128_impl,  //!< Native \c int128_t implementation.
    cairo_impl,   //!< Cairo wideint implementation.
    ld_impl,      //!< `long double` implementation.
  };

  /// Type tag for this implementation.
  static const enum impl_type implementation = int128_impl;

  /// Default constructor.
  inline int64x64_t ()
    : _v (0)  {}
  /**
   * \name Construct from a floating point value.
   *
   * \param [in] value Floating value to represent.
   */
  /**@{*/
  inline int64x64_t (const double value)
  {
    const int64x64_t tmp ((long double)value);
    _v = tmp._v;
  }
  inline int64x64_t (const long double value)
  {
    const bool negative = value < 0;
    const long double v = negative ? -value : value;

    long double fhi;
    long double flo = std::modf (v, &fhi);
    // Add 0.5 to round, which improves the last count
    // This breaks these tests:
    //   TestSuite devices-mesh-dot11s-regression
    //   TestSuite devices-mesh-flame-regression
    //   TestSuite routing-aodv-regression
    //   TestSuite routing-olsr-regression
    // Setting round = 0; breaks:
    //   TestSuite int64x64
    const long double round = 0.5;
    flo = flo * HP_MAX_64 + round;
    int128_t hi = fhi;
    const uint64_t lo = flo;
    if (flo >= HP_MAX_64)
      {
	// conversion to uint64 rolled over
	++hi;
      }
    _v = hi << 64;
    _v |= lo;
    _v = negative ? -_v : _v;
  }
  /**@}*/

  /**
   * \name Construct from an integral type.
   *
   * \param [in] v Integer value to represent.
   */
  /**@{*/
  inline int64x64_t (const int v)
    : _v (v)
  {
    _v <<= 64;
  }
  inline int64x64_t (const long int v)
    : _v (v) 
  {
    _v <<= 64;
  }
  inline int64x64_t (const long long int v)
    : _v (v) 
  {
    _v <<= 64;
  }
  inline int64x64_t (const unsigned int v)
    : _v (v)
  {
    _v <<= 64;
  }
  inline int64x64_t (const unsigned long int v)
    : _v (v) 
  {
    _v <<= 64;
  }
  inline int64x64_t (const unsigned long long int v)
    : _v (v) 
  {
    _v <<= 64;
  }
  /**@}*/
  
  /**
   * Construct from explicit high and low values.
   *
   * \param [in] hi Integer portion.
   * \param [in] lo Fractional portion, already scaled to HP_MAX_64.
   */
  explicit inline int64x64_t (const int64_t hi, const uint64_t lo)
  {
    _v = (int128_t)hi << 64;
    _v |= lo;
  }

  /**
   * Copy constructor.
   *
   * \param [in] o Value to copy.
   */
  inline int64x64_t (const int64x64_t & o)
    : _v (o._v) {}
  /**
   * Assignment.
   *
   * \param [in] o Value to assign to this int64x64_t.
   * \returns This int64x64_t.
   */
  inline int64x64_t & operator = (const int64x64_t & o)
  {
    _v = o._v;
    return *this;
  }

  /**
   * Get this value as a double.
   *
   * \return This value in floating form.
   */
  inline double GetDouble (void) const
  {
    const bool negative = _v < 0;
    const uint128_t value = negative ? -_v : _v;
    const long double fhi = value >> 64;
    const long double flo = (value & HP_MASK_LO) / HP_MAX_64;
    long double retval = fhi;
    retval += flo;
    retval = negative ? -retval : retval;
    return retval;
  }
  /**
   * Get the integer portion.
   *
   * \return The integer portion of this value.
   */
  inline int64_t GetHigh (void) const
  {
    const int128_t retval = _v >> 64;
    return retval;
  }
  /**
   * Get the fractional portion of this value, unscaled.
   *
   * \return The fractional portion, unscaled, as an integer.
   */
  inline uint64_t GetLow (void) const
  {
    const uint128_t retval = _v & HP_MASK_LO;
    return retval;
  }

  /**
   * Multiply this value by a Q0.128 value, presumably representing an inverse,
   * completing a division operation.
   *
   * \param [in] o The inverse operand.
   *
   * \see Invert()
   */
  void MulByInvert (const int64x64_t & o);

  /**
   * Compute the inverse of an integer value.
   *
   * Ordinary division by an integer would be limited to 64 bits of precision.
   * Instead, we multiply by the 128-bit inverse of the divisor.
   * This function computes the inverse to 128-bit precision.
   * MulByInvert() then completes the division.
   *
   * (Really this should be a separate type representing Q0.128.)
   *
   * \param [in] v The value to compute the inverse of.
   * \return A Q0.128 representation of the inverse.
   */
  static int64x64_t Invert (const uint64_t v);

private:

  friend bool         operator == (const int64x64_t & lhs, const int64x64_t & rhs);

  friend bool         operator <  (const int64x64_t & lhs, const int64x64_t & rhs);
  friend bool         operator >  (const int64x64_t & lhs, const int64x64_t & rhs);
  
  friend int64x64_t & operator += (      int64x64_t & lhs, const int64x64_t & rhs);
  friend int64x64_t & operator -= (      int64x64_t & lhs, const int64x64_t & rhs);
  friend int64x64_t & operator *= (      int64x64_t & lhs, const int64x64_t & rhs);
  friend int64x64_t & operator /= (      int64x64_t & lhs, const int64x64_t & rhs);

  friend int64x64_t   operator -  (const int64x64_t & lhs);
  friend int64x64_t   operator !  (const int64x64_t & lhs);

  /**
   * Implement `*=`.
   *
   * \param [in] o The other factor.
   */   
  void Mul (const int64x64_t & o);
  /**
   * Implement `/=`.
   *
   * \param [in] o The divisor.
   */
  void Div (const int64x64_t & o);
  /**
   * Unsigned multiplication of Q64.64 values.
   *
   * Mathematically this should produce a Q128.128 value;
   * we keep the central 128 bits, representing the Q64.64 result.
   * We assert on integer overflow beyond the 64-bit integer portion.
   *
   * \param [in] a First factor.
   * \param [in] b Second factor.
   * \return The Q64.64 product.
   *
   * \internal
   *
   * It might be tempting to just use \pname{a} `*` \pname{b}
   * and be done with it, but it's not that simple.  With \pname{a}
   * and \pname{b} as 128-bit integers, \pname{a} `*` \pname{b}
   * mathematically produces a 256-bit result, which the computer
   * truncates to the lowest 128 bits.  In our case, where \pname{a}
   * and \pname{b} are interpreted as Q64.64 fixed point numbers,
   * the multiplication mathematically produces a Q128.128 fixed point number.
   * We want the middle 128 bits from the result, truncating both the
   * high and low 64 bits.  To achieve this, we carry out the multiplication
   * explicitly with 64-bit operands and 128-bit intermediate results.
   */
  static uint128_t Umul         (const uint128_t a, const uint128_t b);
  /**
   * Unsigned division of Q64.64 values.
   *
   * \param [in] a Numerator.
   * \param [in] b Denominator.
   * \return The Q64.64 representation of `a / b`.
   */
  static uint128_t Udiv         (const uint128_t a, const uint128_t b);
  /**
   * Unsigned multiplication of Q64.64 and Q0.128 values.
   *
   * \param [in] a The numerator, a Q64.64 value.
   * \param [in] b The inverse of the denominator, a Q0.128 value
   * \return The product `a * b`, representing the ration `a / b^-1`.
   *
   * \see Invert()
   */
  static uint128_t UmulByInvert (const uint128_t a, const uint128_t b);

  /**
   * Construct from an integral type.
   *
   * \param [in] v Integer value to represent.
   */
  inline int64x64_t (const int128_t v)
    : _v (v) {}

  int128_t _v;  //!< The Q64.64 value.

};  // class int64x64_t


/**
 * \ingroup highprec
 * Equality operator.
 */
inline bool operator == (const int64x64_t & lhs, const int64x64_t & rhs)
{
  return lhs._v == rhs._v;
}
/**
 * \ingroup highprec
 * Less than operator.
 */
inline bool operator < (const int64x64_t & lhs, const int64x64_t & rhs)
{
  return lhs._v < rhs._v;
}
/**
 * \ingroup highprec
 * Greater operator.
 */
inline bool operator > (const int64x64_t & lhs, const int64x64_t & rhs)
{
  return lhs._v > rhs._v;
}

/**
 * \ingroup highprec
 * Compound addition operator.
 */
inline int64x64_t & operator += (int64x64_t & lhs, const int64x64_t & rhs)
{
  lhs._v += rhs._v;
  return lhs;
}
/**
 * \ingroup highprec
 * Compound subtraction operator.
 */
inline int64x64_t & operator -= (int64x64_t & lhs, const int64x64_t & rhs)
{
  lhs._v -= rhs._v;
  return lhs;
}
/**
 * \ingroup highprec
 * Compound multiplication operator.
 */
inline int64x64_t & operator *= (int64x64_t & lhs, const int64x64_t & rhs)
{
  lhs.Mul (rhs);
  return lhs;
}
/**
 * \ingroup highprec
 * Compound division operator.
 */
inline int64x64_t & operator /= (int64x64_t & lhs, const int64x64_t & rhs)
{
  lhs.Div (rhs);
  return lhs;
}

/**
 * \ingroup highprec
 * Unary plus operator.
 */
inline int64x64_t operator + (const int64x64_t & lhs)
{
  return lhs;
}
/**
 * \ingroup highprec
 * Unary negation operator (change sign operator).
 */
inline int64x64_t operator - (const int64x64_t & lhs)
{
  return int64x64_t (-lhs._v);
}
/**
 * \ingroup highprec
 * Logical not operator.
 */
inline int64x64_t operator ! (const int64x64_t & lhs)
{
  return int64x64_t (!lhs._v);
}


} // namespace ns3

#endif /* INT64X64_128_H */
