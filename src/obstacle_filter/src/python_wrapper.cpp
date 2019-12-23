#include <Python.h>
#include "obstacle_detector/Obstacles.h"
#include "obstacle_filter/arena_filter.h"

int isInArena(double x, double y)
{  
  Point p(x, y);
  return IsPointInAerna(p, 0.2);
}

PyObject* wrap_isInArena(PyObject* self, PyObject* args)
{
  int x, y, result;
  if (! PyArg_ParseTuple(args, "ff:isInArena", &x, &y))
      return NULL;
  result = isInArena(x, y);
  return Py_BuildValue("i", result);
}

static PyMethodDef exampleMethods[] =
{
    {"isInArena", wrap_isInArena, METH_VARARGS, "is in arena!"},
    {NULL, NULL, 0, NULL}
};

extern "C"  //不加会导致找不到initexample
void initlibobstacle_filter()
{
    PyObject* m;
    m = Py_InitModule("libobstacle_filter", exampleMethods);
}
