/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */
// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados_c/external_function_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#include "acados/sim/sim_common.h"
#include "acados/utils/external_function_generic.h"
#include "acados/utils/print.h"


// example specific
#include "ACADOS_model_model/ACADOS_model_model.h"
#include "acados_sim_solver_ACADOS_model.h"


// ** solver data **

ACADOS_model_sim_solver_capsule * ACADOS_model_acados_sim_solver_create_capsule()
{
    void* capsule_mem = malloc(sizeof(ACADOS_model_sim_solver_capsule));
    ACADOS_model_sim_solver_capsule *capsule = (ACADOS_model_sim_solver_capsule *) capsule_mem;

    return capsule;
}


int ACADOS_model_acados_sim_solver_free_capsule(ACADOS_model_sim_solver_capsule * capsule)
{
    free(capsule);
    return 0;
}


int ACADOS_model_acados_sim_create(ACADOS_model_sim_solver_capsule * capsule)
{
    // initialize
    const int nx = ACADOS_MODEL_NX;
    const int nu = ACADOS_MODEL_NU;
    const int nz = ACADOS_MODEL_NZ;
    const int np = ACADOS_MODEL_NP;
    bool tmp_bool;

    
    double Tsim = 0.002;

    
    capsule->sim_impl_dae_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_impl_dae_fun_jac_x_xdot_z = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_impl_dae_jac_x_xdot_u_z = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    // external functions (implicit model)
    capsule->sim_impl_dae_fun->casadi_fun = &ACADOS_model_impl_dae_fun;
    capsule->sim_impl_dae_fun->casadi_work = &ACADOS_model_impl_dae_fun_work;
    capsule->sim_impl_dae_fun->casadi_sparsity_in = &ACADOS_model_impl_dae_fun_sparsity_in;
    capsule->sim_impl_dae_fun->casadi_sparsity_out = &ACADOS_model_impl_dae_fun_sparsity_out;
    capsule->sim_impl_dae_fun->casadi_n_in = &ACADOS_model_impl_dae_fun_n_in;
    capsule->sim_impl_dae_fun->casadi_n_out = &ACADOS_model_impl_dae_fun_n_out;
    external_function_param_casadi_create(capsule->sim_impl_dae_fun, np);

    capsule->sim_impl_dae_fun_jac_x_xdot_z->casadi_fun = &ACADOS_model_impl_dae_fun_jac_x_xdot_z;
    capsule->sim_impl_dae_fun_jac_x_xdot_z->casadi_work = &ACADOS_model_impl_dae_fun_jac_x_xdot_z_work;
    capsule->sim_impl_dae_fun_jac_x_xdot_z->casadi_sparsity_in = &ACADOS_model_impl_dae_fun_jac_x_xdot_z_sparsity_in;
    capsule->sim_impl_dae_fun_jac_x_xdot_z->casadi_sparsity_out = &ACADOS_model_impl_dae_fun_jac_x_xdot_z_sparsity_out;
    capsule->sim_impl_dae_fun_jac_x_xdot_z->casadi_n_in = &ACADOS_model_impl_dae_fun_jac_x_xdot_z_n_in;
    capsule->sim_impl_dae_fun_jac_x_xdot_z->casadi_n_out = &ACADOS_model_impl_dae_fun_jac_x_xdot_z_n_out;
    external_function_param_casadi_create(capsule->sim_impl_dae_fun_jac_x_xdot_z, np);

    // external_function_param_casadi impl_dae_jac_x_xdot_u_z;
    capsule->sim_impl_dae_jac_x_xdot_u_z->casadi_fun = &ACADOS_model_impl_dae_jac_x_xdot_u_z;
    capsule->sim_impl_dae_jac_x_xdot_u_z->casadi_work = &ACADOS_model_impl_dae_jac_x_xdot_u_z_work;
    capsule->sim_impl_dae_jac_x_xdot_u_z->casadi_sparsity_in = &ACADOS_model_impl_dae_jac_x_xdot_u_z_sparsity_in;
    capsule->sim_impl_dae_jac_x_xdot_u_z->casadi_sparsity_out = &ACADOS_model_impl_dae_jac_x_xdot_u_z_sparsity_out;
    capsule->sim_impl_dae_jac_x_xdot_u_z->casadi_n_in = &ACADOS_model_impl_dae_jac_x_xdot_u_z_n_in;
    capsule->sim_impl_dae_jac_x_xdot_u_z->casadi_n_out = &ACADOS_model_impl_dae_jac_x_xdot_u_z_n_out;
    external_function_param_casadi_create(capsule->sim_impl_dae_jac_x_xdot_u_z, np);

    

    // sim plan & config
    sim_solver_plan_t plan;
    plan.sim_solver = IRK;

    // create correct config based on plan
    sim_config * ACADOS_model_sim_config = sim_config_create(plan);
    capsule->acados_sim_config = ACADOS_model_sim_config;

    // sim dims
    void *ACADOS_model_sim_dims = sim_dims_create(ACADOS_model_sim_config);
    capsule->acados_sim_dims = ACADOS_model_sim_dims;
    sim_dims_set(ACADOS_model_sim_config, ACADOS_model_sim_dims, "nx", &nx);
    sim_dims_set(ACADOS_model_sim_config, ACADOS_model_sim_dims, "nu", &nu);
    sim_dims_set(ACADOS_model_sim_config, ACADOS_model_sim_dims, "nz", &nz);


    // sim opts
    sim_opts *ACADOS_model_sim_opts = sim_opts_create(ACADOS_model_sim_config, ACADOS_model_sim_dims);
    capsule->acados_sim_opts = ACADOS_model_sim_opts;
    int tmp_int = 3;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "newton_iter", &tmp_int);
    double tmp_double = 0;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "newton_tol", &tmp_double);
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "collocation_type", &collocation_type);


    tmp_int = 4;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "num_stages", &tmp_int);
    tmp_int = 4;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "num_steps", &tmp_int);

    // options that are not available to AcadosOcpSolver
    //  (in OCP they will be determined by other options, like exact_hessian)
    tmp_bool = true;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "sens_forw", &tmp_bool);
    tmp_bool = false;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "sens_adj", &tmp_bool);
    tmp_bool = false;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "sens_algebraic", &tmp_bool);
    tmp_bool = false;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "sens_hess", &tmp_bool);
    tmp_bool = true;
    sim_opts_set(ACADOS_model_sim_config, ACADOS_model_sim_opts, "output_z", &tmp_bool);



    // sim in / out
    sim_in *ACADOS_model_sim_in = sim_in_create(ACADOS_model_sim_config, ACADOS_model_sim_dims);
    capsule->acados_sim_in = ACADOS_model_sim_in;
    sim_out *ACADOS_model_sim_out = sim_out_create(ACADOS_model_sim_config, ACADOS_model_sim_dims);
    capsule->acados_sim_out = ACADOS_model_sim_out;

    sim_in_set(ACADOS_model_sim_config, ACADOS_model_sim_dims,
               ACADOS_model_sim_in, "T", &Tsim);

    // model functions
    ACADOS_model_sim_config->model_set(ACADOS_model_sim_in->model,
                 "impl_ode_fun", capsule->sim_impl_dae_fun);
    ACADOS_model_sim_config->model_set(ACADOS_model_sim_in->model,
                 "impl_ode_fun_jac_x_xdot", capsule->sim_impl_dae_fun_jac_x_xdot_z);
    ACADOS_model_sim_config->model_set(ACADOS_model_sim_in->model,
                 "impl_ode_jac_x_xdot_u", capsule->sim_impl_dae_jac_x_xdot_u_z);

    // sim solver
    sim_solver *ACADOS_model_sim_solver = sim_solver_create(ACADOS_model_sim_config,
                                               ACADOS_model_sim_dims, ACADOS_model_sim_opts);
    capsule->acados_sim_solver = ACADOS_model_sim_solver;


    /* initialize parameter values */
    double* p = calloc(np, sizeof(double));
    

    ACADOS_model_acados_sim_update_params(capsule, p, np);
    free(p);


    /* initialize input */
    // x
    double x0[10];
    for (int ii = 0; ii < 10; ii++)
        x0[ii] = 0.0;

    sim_in_set(ACADOS_model_sim_config, ACADOS_model_sim_dims,
               ACADOS_model_sim_in, "x", x0);


    // u
    double u0[4];
    for (int ii = 0; ii < 4; ii++)
        u0[ii] = 0.0;

    sim_in_set(ACADOS_model_sim_config, ACADOS_model_sim_dims,
               ACADOS_model_sim_in, "u", u0);

    // S_forw
    double S_forw[140];
    for (int ii = 0; ii < 140; ii++)
        S_forw[ii] = 0.0;
    for (int ii = 0; ii < 10; ii++)
        S_forw[ii + ii * 10 ] = 1.0;


    sim_in_set(ACADOS_model_sim_config, ACADOS_model_sim_dims,
               ACADOS_model_sim_in, "S_forw", S_forw);

    int status = sim_precompute(ACADOS_model_sim_solver, ACADOS_model_sim_in, ACADOS_model_sim_out);

    return status;
}


int ACADOS_model_acados_sim_solve(ACADOS_model_sim_solver_capsule *capsule)
{
    // integrate dynamics using acados sim_solver
    int status = sim_solve(capsule->acados_sim_solver,
                           capsule->acados_sim_in, capsule->acados_sim_out);
    if (status != 0)
        printf("error in ACADOS_model_acados_sim_solve()! Exiting.\n");

    return status;
}


void ACADOS_model_acados_sim_batch_solve(ACADOS_model_sim_solver_capsule ** capsules, int N_batch)
{

    for (int i = 0; i < N_batch; i++)
    {
        sim_solve(capsules[i]->acados_sim_solver, capsules[i]->acados_sim_in, capsules[i]->acados_sim_out);
    }


    return;
}


int ACADOS_model_acados_sim_free(ACADOS_model_sim_solver_capsule *capsule)
{
    // free memory
    sim_solver_destroy(capsule->acados_sim_solver);
    sim_in_destroy(capsule->acados_sim_in);
    sim_out_destroy(capsule->acados_sim_out);
    sim_opts_destroy(capsule->acados_sim_opts);
    sim_dims_destroy(capsule->acados_sim_dims);
    sim_config_destroy(capsule->acados_sim_config);

    // free external function
    external_function_param_casadi_free(capsule->sim_impl_dae_fun);
    external_function_param_casadi_free(capsule->sim_impl_dae_fun_jac_x_xdot_z);
    external_function_param_casadi_free(capsule->sim_impl_dae_jac_x_xdot_u_z);
    free(capsule->sim_impl_dae_fun);
    free(capsule->sim_impl_dae_fun_jac_x_xdot_z);
    free(capsule->sim_impl_dae_jac_x_xdot_u_z);

    return 0;
}


int ACADOS_model_acados_sim_update_params(ACADOS_model_sim_solver_capsule *capsule, double *p, int np)
{
    int status = 0;
    int casadi_np = ACADOS_MODEL_NP;

    if (casadi_np != np) {
        printf("ACADOS_model_acados_sim_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    capsule->sim_impl_dae_fun[0].set_param(capsule->sim_impl_dae_fun, p);
    capsule->sim_impl_dae_fun_jac_x_xdot_z[0].set_param(capsule->sim_impl_dae_fun_jac_x_xdot_z, p);
    capsule->sim_impl_dae_jac_x_xdot_u_z[0].set_param(capsule->sim_impl_dae_jac_x_xdot_u_z, p);

    return status;
}

/* getters pointers to C objects*/
sim_config * ACADOS_model_acados_get_sim_config(ACADOS_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_config;
};

sim_in * ACADOS_model_acados_get_sim_in(ACADOS_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_in;
};

sim_out * ACADOS_model_acados_get_sim_out(ACADOS_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_out;
};

void * ACADOS_model_acados_get_sim_dims(ACADOS_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_dims;
};

sim_opts * ACADOS_model_acados_get_sim_opts(ACADOS_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_opts;
};

sim_solver  * ACADOS_model_acados_get_sim_solver(ACADOS_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_solver;
};

