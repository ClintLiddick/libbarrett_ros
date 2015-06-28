#ifndef LIBBARRETT_ROS_PARAMS_H_
#define LIBBARRETT_ROS_PARAMS_H_
#include <XmlRpcValue.h>

namespace libbarrett_ros {

using ::XmlRpc::XmlRpcValue;

template <class T>
struct XmlRpcMetadata {
};

template <>
struct XmlRpcMetadata<bool> {
  typedef bool element_type;
  typedef element_type const &return_type;

  static XmlRpcValue::Type enum_type() { return XmlRpcValue::TypeBoolean; }
  char const *name() const { return "boolean"; }
};

template <>
struct XmlRpcMetadata<int> {
  typedef int element_type;
  typedef element_type const &return_type;

  static XmlRpcValue::Type enum_type() { return XmlRpcValue::TypeInt; }
  char const *name() const { return "integer"; }
};

template <>
struct XmlRpcMetadata<double> {
  typedef int element_type;
  typedef element_type const &return_type;

  static XmlRpcValue::Type enum_type() { return XmlRpcValue::TypeDouble; }
  char const *name() const { return "double"; }
};

template <>
struct XmlRpcMetadata<std::string> {
  typedef std::string element_type;
  typedef element_type const &return_type;

  static XmlRpcValue::Type enum_type() { return XmlRpcValue::TypeString; }
  char const *name() const { return "string"; }
};

template <class T>
struct get_value_impl {
  typedef T element_type;
  typedef element_type const &return_type;

  static return_type call(XmlRpcValue const &value_xmlrpc)
  {
    if (value_xmlrpc.getType() == XmlRpcMetadata<T>::enum_type()) {
      return static_cast<T &>(const_cast<XmlRpcValue &>(value_xmlrpc));
    } else {
      throw std::runtime_error("Parameter has incorrect type.");
    }
  }
};

template <class T>
struct get_value_impl<std::vector<T> > {
  typedef std::vector<T> element_type;
  typedef element_type return_type;
  
  static return_type call(XmlRpcValue const &vector_xmlrpc)
  {
    if (vector_xmlrpc.getType() != XmlRpcValue::TypeArray) {
      throw std::runtime_error("Parameter is not an array.");
    }

    std::vector<T> output_vector;
    output_vector.reserve(vector_xmlrpc.size());

    for (size_t i = 0; i < vector_xmlrpc.size(); ++i) {
      output_vector.push_back(get_value_impl<T>::call(vector_xmlrpc[i]));
    }
    return output_vector;
  }
};

template <class T, size_t N>
struct get_value_impl<boost::array<T, N> > {
  typedef boost::array<T, N> element_type;
  typedef element_type return_type;
  
  static return_type call(XmlRpcValue const &vector_xmlrpc)
  {
    if (vector_xmlrpc.getType() != XmlRpcValue::TypeArray) {
      throw std::runtime_error("Parameter is not an array.");
    } else if (vector_xmlrpc.size() != N) {
      throw std::runtime_error("Parameter has incorrect length.");
    }

    boost::array<T, N> output_array;
    for (size_t i = 0; i < vector_xmlrpc.size(); ++i) {
      output_array[i] = get_value_impl<T>::call(vector_xmlrpc[i]);
    }
    return output_array;
  }
};

template <class T>
typename get_value_impl<T>::return_type get_value(XmlRpcValue const &xmlrpc)
{
  return get_value_impl<T>::call(xmlrpc);
}

template <class T>
typename get_value_impl<T>::return_type get_or_throw(
  XmlRpcValue const &struct_xmlrpc, std::string const &key)
{
  if (struct_xmlrpc.getType() != XmlRpcValue::TypeStruct) {
    throw std::runtime_error("Parameter is not a struct.");
  } else if (!struct_xmlrpc.hasMember(key)) {
    throw std::runtime_error("Required parameter is missing.");
  } else {
    return get_value<T>(const_cast<XmlRpcValue &>(struct_xmlrpc)[key]);
  }
}

template <class T>
typename get_value_impl<T>::return_type get_or_default(
  XmlRpcValue const &struct_xmlrpc, std::string const &key,
  T const &default_value)
{
  if (struct_xmlrpc.getType() != XmlRpcValue::TypeStruct) {
    throw std::runtime_error("Parameter is not a struct.");
  } else if (!struct_xmlrpc.hasMember(key)) {
    return default_value;
  } else {
    return get_value<T>(const_cast<XmlRpcValue &>(struct_xmlrpc)[key]);
  }
}

}

#endif
