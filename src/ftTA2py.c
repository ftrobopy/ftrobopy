#include "Python.h"
#include <stdlib.h>
#include <stdio.h>

#include "KeLibTxtDl.h"   // TXT Lib
#include "FtShmem.h"      // TXT Transfer Area

//////////////////////////////////////////////////////////////////////////////////////////
// ftTA2py -- part of ftrobopy; module to use Transfer Area method in download mode
// fischertechnik TXT controller
// version 0.10 (2020-05-27)
// by Torsten Stuehn <stuehn@mailbox.org>
//
// The MIT License (MIT)
//
// Copyright (c) 2020 Torsten Stuehn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


FISH_X1_TRANSFER   *pTArea;
int isInitialized = 0;
unsigned int DebugFlags;
FILE *DebugFile;

struct module_state {
  PyObject *error;
};

#define GETSTATE(m) ((struct module_state*)PyModule_GetState(m))

static PyObject *ftTA2py_initTA(PyObject *self, PyObject *args);
static PyObject *ftTA2py_stopTA(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1config_uni(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_uni(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_cnt_in(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_counter(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_cnt_resetted(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_motor_ex_reached(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_cnt_reset_cmd_id(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1in_motor_ex_cmd_id(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1out_cnt_reset_cmd_id(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1out_master(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1out_distance(PyObject *self, PyObject *args);
// static PyObject *ftTA2py_fX1out_motor_ex_cmd_id(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1out_incr_motor_cmd_id(PyObject *self, PyObject *args);
static PyObject *ftTA2py_fX1out_duty(PyObject *self, PyObject *args);

static PyMethodDef ftTA2py_methods[] = {
  {"initTA", ftTA2py_initTA, METH_VARARGS, "Initialize Transfer Area program."},
  {"stopTA", ftTA2py_stopTA, METH_VARARGS, "Stop Transfer Area program."},
  {"fX1config_uni", ftTA2py_fX1config_uni, METH_VARARGS, "Configure universal input."},
  {"fX1in_uni", ftTA2py_fX1in_uni, METH_VARARGS, "Current value of universal input."},
  {"fX1in_cnt_in", ftTA2py_fX1in_cnt_in, METH_VARARGS, "Logic state counter input."},
  {"fX1in_counter", ftTA2py_fX1in_counter, METH_VARARGS, "Current counter value."},
  {"fX1in_cnt_resetted", ftTA2py_fX1in_cnt_resetted, METH_VARARGS, "Set to 1 when last requested counter reset was fulfilled."},
  {"fX1in_motor_ex_reached", ftTA2py_fX1in_motor_ex_reached, METH_VARARGS, "Set to 1 by motor control if target position is reached."},
  {"fX1in_cnt_reset_cmd_id", ftTA2py_fX1in_cnt_reset_cmd_id, METH_VARARGS, "Counter reset command id of the last fulfilled counter reset."},
  {"fX1in_motor_ex_cmd_id", ftTA2py_fX1in_motor_ex_cmd_id, METH_VARARGS, "Motor extended command id of the last fulfilled motor_ex command."},
    
  {"fX1out_cnt_reset_cmd_id", ftTA2py_fX1out_cnt_reset_cmd_id, METH_VARARGS, "Counter reset requests (increment each time by one)."},
  {"fX1out_master", ftTA2py_fX1out_master, METH_VARARGS, "If not 0, synchronize this channel with the given channel (1:channel 0, ..)."},
  {"fX1out_distance", ftTA2py_fX1out_distance, METH_VARARGS, "Set distance to drive motor."},
  //{"fX1out_motor_ex_cmd_id", ftTA2py_fX1out_motor_ex_cmd_id, METH_VARARGS, "Increments motor_ex settings change. "},
  {"fX1out_incr_motor_cmd_id", ftTA2py_fX1out_incr_motor_cmd_id, METH_VARARGS, "Increment motor_cmd_id. Necessary after each motor distance setting. "},
  {"fX1out_duty", ftTA2py_fX1out_duty, METH_VARARGS, "Set PWM duty cycle value for motor."},
    
  {NULL, NULL, 0, NULL}
};

static int ftTA2py_traverse(PyObject *m, visitproc visit, void *arg) {
    Py_VISIT(GETSTATE(m)->error);
    return 0;
}
static int ftTA2py_clear(PyObject *m) {
    Py_CLEAR(GETSTATE(m)->error);
    return 0;
}
static struct PyModuleDef moduledef = {
        PyModuleDef_HEAD_INIT,
        "ftTA2py",
        NULL,
        sizeof(struct module_state),
        ftTA2py_methods,
        NULL,
        ftTA2py_traverse,
        ftTA2py_clear,
        NULL
};
#define INITERROR return NULL

PyMODINIT_FUNC
PyInit_ftTA2py(void)
{
    PyObject *module = PyModule_Create(&moduledef);
    if (module == NULL)
        INITERROR;
    struct module_state *st = GETSTATE(module);
    st->error = PyErr_NewException("ftTA2py.error", NULL, NULL);
    if (st->error == NULL) {
        Py_DECREF(module);
        INITERROR;
    }
    return module;
}

// Initialize Transfer Area program
static PyObject *
ftTA2py_initTA(PyObject *self, PyObject *args)
{
  PyObject * res;

  if (isInitialized == 1) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  
  if (StartTxtDownloadProg() == KELIB_ERROR_NONE) {
    pTArea = GetKeLibTransferAreaMainAddress();
    if (pTArea) {
      isInitialized = 1;
    }
  }

  // returns 
  res = Py_BuildValue("I", isInitialized);
  return res;
}

// Stop Transfer Area program
static PyObject *
ftTA2py_stopTA(PyObject *self, PyObject *args)
{
  PyObject * res;

  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  StopTxtDownloadProg();
  isInitialized = 0;
  pTArea = NULL;
  res = Py_BuildValue("I", 0);
  return res;
}

//Configure universal input
static PyObject *
ftTA2py_fX1config_uni(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned char mode;    // 0:MODE_U, 1:MODE_R, 2:MODE_R2, 3:MODE_ULTRASONIC, 4:MODE_INVALID
  unsigned char digital; // 0:Analog, 1:Digital
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BBBB", &extnr, &inputnr, &mode, &digital))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  (pTArea + extnr)->ftX1config.uni[inputnr].mode = mode;
  (pTArea + extnr)->ftX1config.uni[inputnr].digital = digital;
  (pTArea + extnr)->ftX1state.config_id++;

  Py_INCREF(Py_None);
  return Py_None;
}

// Current value of universal input
static PyObject *
ftTA2py_fX1in_uni(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned int value;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  value = (pTArea + extnr)->ftX1in.uni[inputnr];
    
  res = Py_BuildValue("I", value);
  return res;
}

// Logic state counter input
static PyObject *
ftTA2py_fX1in_cnt_in(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned int value;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  value = (pTArea + extnr)->ftX1in.cnt_in[inputnr];
    
  res = Py_BuildValue("I", value);
  return res;
}

// Current counter value
static PyObject *
ftTA2py_fX1in_counter(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned int value;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  value = (pTArea + extnr)->ftX1in.counter[inputnr];
    
  res = Py_BuildValue("I", value);
  return res;
}

// Set to 1 when last requested counter reset was fulfilled
static PyObject *
ftTA2py_fX1in_cnt_resetted(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned int value;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  value = (pTArea + extnr)->ftX1in.cnt_resetted[inputnr];
    
  res = Py_BuildValue("I", value);
  return res;
}

// Set to 1 by motor control if target position is reached
static PyObject *
ftTA2py_fX1in_motor_ex_reached(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned int value;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  value = (pTArea + extnr)->ftX1in.motor_ex_reached[inputnr];
    
  res = Py_BuildValue("I", value);
  return res;
}

// Counter reset command id of the last fulfilled counter reset
static PyObject *
ftTA2py_fX1in_cnt_reset_cmd_id(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  unsigned int value;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  value = (pTArea + extnr)->ftX1in.cnt_reset_cmd_id[inputnr];
    
  res = Py_BuildValue("I", value);
  return res;
}

// Motor extended command id of the last fulfilled motor_ex command
static PyObject *
ftTA2py_fX1in_motor_ex_cmd_id(PyObject *self, PyObject *args)
{
  unsigned char extnr;   // 0:Master, 1:Extension
  unsigned char inputnr; // 0-7:Input1-Input8
  int x;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &inputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  x = (pTArea + extnr)->ftX1in.motor_ex_cmd_id[inputnr];
    
  res = Py_BuildValue("I", x);
  return res;
}

// Counter reset requests (increment each time by one)
static PyObject *
ftTA2py_fX1out_cnt_reset_cmd_id(PyObject *self, PyObject *args)
{
  unsigned char extnr;    // 0:Master, 1:Extension
  unsigned char outputnr; // 0-7:Output1-Output8
  unsigned int distance;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BBI", &extnr, &outputnr, &distance))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

    (pTArea + extnr)->ftX1out.distance[outputnr] = distance;
    Py_INCREF(Py_None);
    return Py_None;
}

// If not 0, synchronize this channel with the given channel (1:channel 0, ..)
static PyObject *
ftTA2py_fX1out_master(PyObject *self, PyObject *args)
{
  unsigned char extnr;    // 0:Master, 1:Extension
  unsigned char outputnr; // 0-7:Output1-Output8
  unsigned int master;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BBI", &extnr, &outputnr, &master))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

    (pTArea + extnr)->ftX1out.master[outputnr] = master;
    Py_INCREF(Py_None);
    return Py_None;
}

// Set distance of motor to drive
static PyObject *
ftTA2py_fX1out_distance(PyObject *self, PyObject *args)
{
  unsigned char extnr;    // 0:Master, 1:Extension
  unsigned char outputnr; // 0-7:Output1-Output8
  unsigned int distance;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BBI", &extnr, &outputnr, &distance))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

    (pTArea + extnr)->ftX1out.distance[outputnr] = distance;
    Py_INCREF(Py_None);
    return Py_None;
}

// Increment motor_cmd_id. Necessary after each motor distance setting
static PyObject *
ftTA2py_fX1out_incr_motor_cmd_id(PyObject *self, PyObject *args)
{
  unsigned char extnr;    // 0:Master, 1:Extension
  unsigned char outputnr; // 0-7:Output1-Output8
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BB", &extnr, &outputnr))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

    (pTArea + extnr)->ftX1out.motor_ex_cmd_id[outputnr]++;
    (pTArea + extnr)->ftX1out.motor_ex_cmd_id[outputnr] &= 0x07;
    Py_INCREF(Py_None);
    return Py_None;
}

// Set PWM duty cycle value for motor
static PyObject *
ftTA2py_fX1out_duty(PyObject *self, PyObject *args)
{
  unsigned char extnr;    // 0:Master, 1:Extension
  unsigned char outputnr; // 0-7:Output1-Output8
  unsigned int duty;
    
  PyObject * res;

  if (!PyArg_ParseTuple(args, "BBI", &extnr, &outputnr, &duty))
    return NULL;
  if (isInitialized == 0) {
    Py_INCREF(Py_None);
    return Py_None;
  }

    (pTArea + extnr)->ftX1out.duty[outputnr] = duty;
    Py_INCREF(Py_None);
    return Py_None;
}


//////////////////////////////////////////////////////////////////////////////////////////
// End of ftTA2py
//////////////////////////////////////////////////////////////////////////////////////////

