#define S_FUNCTION_NAME  casadi_fun
#define S_FUNCTION_LEVEL 2

#include <casadi/casadi_c.h>

#include "simstruc.h"

static int id = -1;
static int ret = -1;

static casadi_int n_in, n_out;
static casadi_int sz_arg, sz_res, sz_iw, sz_w;

static int mem;


void cleanup() {
  if (ret==0) {
    casadi_c_pop();
    ret = -1;
  }
}

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;
    const casadi_int* sp;
    const char *file_name;
    const char *function_name;
    ssSetNumSFcnParams(S, 2);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!mxIsChar(ssGetSFcnParam(S, 0))) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "file name must be a string.");
    }
    if (!mxIsChar(ssGetSFcnParam(S, 1))) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "function name must be a string.");
    }
    
    file_name = mxArrayToString(ssGetSFcnParam(S, 0));

    if (!file_name) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "file name must be a string.");
    }
    function_name = mxArrayToString(ssGetSFcnParam(S, 1));
    if (!function_name) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "function name must be a string.");
    }

    // Simulink does not provide a cleanup-hook when parameters are changed
    cleanup();

    // Load file
    mexPrintf("Loading file '%s'...", file_name);
    ret = casadi_c_push_file(file_name);
    mxFree(file_name);

    if (ret) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:Load",
                         "Failed to load file.");
    }
    mexPrintf("success\n");

    // Load function
    mexPrintf("Locating function '%s'...", function_name);
    id = casadi_c_id(function_name);
    mxFree(function_name);
    if (id<0) {
      casadi_c_pop();
      mexErrMsgIdAndTxt( "MATLAB:s_function:Load",
                         "Failed to locate function in loaded file.");
    }
    mexPrintf("success\n");


    /* Read in CasADi function dimensions */
    n_in = casadi_c_n_in_id(id);
    n_out = casadi_c_n_out_id(id);
    casadi_c_work_id(id, &sz_arg, &sz_res, &sz_iw, &sz_w);
    
    /* Set up simulink input/output ports */
    if (!ssSetNumInputPorts(S, n_in)) return;
    for (i=0;i<n_in;++i) {
       sp = casadi_c_sparsity_in_id(id, i);
      /* Dense inputs assumed here */
      ssSetInputPortDirectFeedThrough(S, i, 1);
      casadi_int nnz = sp[2+sp[1]];
      if (nnz!=sp[0]*sp[1]) {
        casadi_c_pop();
        mexErrMsgIdAndTxt( "MATLAB:s_function:sparsity",
                         "This example only supports dense inputs.");
      }
      ssSetInputPortMatrixDimensions(S, i, sp[0], sp[1]);
    }

    if (!ssSetNumOutputPorts(S, n_out)) return;
    for (i=0;i<n_out;++i) {
      sp = casadi_c_sparsity_out_id(id, i);
      casadi_int nnz = sp[2+sp[1]];
      /* Dense outputs assumed here */
      if (nnz!=sp[0]*sp[1]) {
        casadi_c_pop();
        mexErrMsgIdAndTxt( "MATLAB:s_function:sparsity",
                         "This example only supports dense outputs. Use 'densify'.");
      }
      ssSetOutputPortMatrixDimensions(S, i, sp[0], sp[1]);
    }

    ssSetNumSampleTimes(S, 1);
    
    /* Set up CasADi function work vector sizes */
    ssSetNumRWork(S, sz_w);
    ssSetNumIWork(S, sz_iw*sizeof(casadi_int)/sizeof(int_T));
    ssSetNumPWork(S, sz_arg+sz_res);
    ssSetNumNonsampledZCs(S, 0);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    // Make sure mdlTerminate is called on error
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    void** p;
    const real_T** arg;
    double* w;
    casadi_int* iw;
    int_T i;

    /* Set up CasADi function work vectors */
    p = ssGetPWork(S);
    arg = (const real_T**) p;
    p += sz_arg;
    real_T** res = (real_T**) p;
    w = ssGetRWork(S);
    iw = (casadi_int*) ssGetIWork(S);
    
    
    /* Point to input and output buffers */  
    for (i=0; i<n_in;++i) {
      arg[i] = *ssGetInputPortRealSignalPtrs(S,i);
    }
    for (i=0; i<n_out;++i) {
      res[i] = ssGetOutputPortRealSignal(S,i);
    }

    /* Run the CasADi function */
    if (casadi_c_eval_id(id, arg, res, iw, w, mem)) {
      ssPrintf("Failed to evaluate\n");
    }
}

static void mdlStart(SimStruct *S)
{
    // Allocate memory (thread-safe)
    casadi_c_incref_id(id);
    // Checkout thread-local memory (not thread-safe)
    mem = casadi_c_checkout_id(id);
}

static void mdlTerminate(SimStruct *S) {
  /* Free memory (thread-safe) */
  casadi_c_decref_id(id);
  // Release thread-local (not thread-safe)
  casadi_c_release_id(id, mem);

  cleanup();
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif