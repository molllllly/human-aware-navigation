%MappedType std::string
{
%TypeHeaderCode
#include <string>
%End

%ConvertFromTypeCode
    // convert an std::string to a Python string
    // I removed the part that converts to a Python unicode string.
    PyObject* newstring;
    newstring = PyUnicode_FromString(sipCpp->c_str());
    return newstring;
%End

%ConvertToTypeCode
    // Allow a Python string (or a unicode string) whenever a string is
    // expected.
    // If argument is a Unicode string, just decode it to UTF-8
    // If argument is a Python string, assume it's UTF-8
    if (sipIsErr == NULL)
    {
        return (PyUnicode_Check(sipPy));
    }
    if (sipPy == Py_None)
    {
        *sipCppPtr = new std::string;
        return 1;
    }
    if (PyUnicode_Check(sipPy))
    {
        PyObject* s = PyUnicode_AsEncodedString(sipPy, "UTF-8", "");
        *sipCppPtr = new std::string(PyBytes_AS_STRING(s));
        Py_DECREF(s);
        return 1;
    }
    return 0;
%End
};

// SIP support for std::vector 
// by Giovanni Bajo <rasky at develer.com>
// Public domain

// ****************************************************
// SIP generic implementation for std::vector<>
// ****************************************************
// ALas, this template-based generic implementation is valid only
// if the element type is a SIP-wrapped type. For basic types (int, double, etc.) 
// we are forced to cut & paste to provide a specialization.
// source: https://www.riverbankcomputing.com/pipermail/pyqt/2007-March/015780.html


template<TYPE>
%MappedType std::vector<TYPE>
{
%TypeHeaderCode
#include <vector>
%End

%ConvertFromTypeCode
    PyObject *l;

    // Create the Python list of the correct length.
    if ((l = PyList_New(sipCpp -> size())) == NULL)
        return NULL;

    // Go through each element in the C++ instance and convert it to a
    // wrapped P2d.
    for (int i = 0; i < (int)sipCpp -> size(); ++i)
    {
        TYPE *cpp = new TYPE(sipCpp -> at(i));
        PyObject *pobj;

        // Get the Python wrapper for the Type instance, creating a new
        // one if necessary, and handle any ownership transfer.
        if ((pobj = sipConvertFromInstance(cpp, sipClass_TYPE, sipTransferObj)) == NULL)
        {
            // There was an error so garbage collect the Python list.
            Py_DECREF(l);
            return NULL;
        }

        // Add the wrapper to the list.
        PyList_SET_ITEM(l, i, pobj);
    }

    // Return the Python list.
    return l;
%End

%ConvertToTypeCode
    // Check if type is compatible
    if (sipIsErr == NULL)
    {
        // Must be any iterable
        PyObject *i = PyObject_GetIter(sipPy);
        bool iterable = (i != NULL);
        Py_XDECREF(i);
        return iterable;
    }

    // Iterate over the object
    PyObject *iterator = PyObject_GetIter(sipPy);
    PyObject *item;

    std::vector<TYPE> *V = new std::vector<TYPE>();

    while ((item = PyIter_Next(iterator)))
    {
        if (!sipCanConvertToInstance(item, sipClass_TYPE, SIP_NOT_NONE))
        {
            PyErr_Format(PyExc_TypeError, "object in iterable cannot be converted to TYPE");
            *sipIsErr = 1;
            break;
        }

        int state;
        TYPE* p = reinterpret_cast<TYPE*>(
             sipConvertToInstance(item, sipClass_TYPE, 0, SIP_NOT_NONE, &state, sipIsErr));

        if (!*sipIsErr)
            V->push_back(*p);

        sipReleaseInstance(p, sipClass_TYPE, state);
        Py_DECREF(item);
    }

    Py_DECREF(iterator);

    if (*sipIsErr)
    {
        delete V;
        return 0;
    }

    *sipCppPtr = V;
    return sipGetState(sipTransferObj);
%End
};