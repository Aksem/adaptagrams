/**********************************************************
 * Based on constrained_majorization.c
 *
 * Perform stress majorization subject
 * to separation constraints.
 *
 * Available separation constraints so far are:
 *  o Directed edge constraints
 *  o Node non-overlap constraints
 *  o Cluster containment constraints
 *  o Cluster/node non-overlap constraints
 *
 * Tim Dwyer, 2006
 **********************************************************/

#include "digcola.h"
#ifdef DIGCOLA
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <float.h>
#include "stress.h"
#include "dijkstra.h"
#include "bfs.h"
#include "matrix_ops.h"
#include "kkutils.h"
#include "conjgrad.h"
#include <csolve_VPSC.h>
#include "quad_prog_vpsc.h"
#include "quad_prog_solver.h"
#include "matrix_ops.h"

#define localConstrMajorIterations 100

int 
stress_majorization_vsep(
    vtx_data* graph,    /* Input graph in sparse representation	 */
    int n,              /* Number of nodes */
    int nedges_graph,   /* Number of edges */
    double** d_coords,  /* Coordinates of nodes (output layout)  */
    int dim,            /* Dimemsionality of layout */
    int model,          /* difference model */
    int maxi,           /* max iterations */
    int diredges,       /* 1=generate directed edge constraints */
    double edge_gap,    /* amount to force vertical separation of */
                        /* start/end nodes */
    int noverlap,       /* 1=generate non-overlap constraints */
                        /* 2=remove overlaps after layout */
    pointf gap,        /* hor and vert gap to enforce when removing overlap*/
    pointf* nsize,     /* node widths and heights */
    cluster_data* clusters
                        /* list of node indices for each cluster */
)
{
    int iterations = 0;    /* Output: number of iteration of the process */

	/*************************************************
	** Computation of full, dense, unrestricted k-D ** 
	** stress minimization by majorization          ** 
	** This function imposes HIERARCHY CONSTRAINTS  **
	*************************************************/

	int i,j,k;
	float * lap1 = NULL;
	float * dist_accumulator = NULL;
	float * tmp_coords = NULL;
	float ** b = NULL;
	double * degrees = NULL;
	float * lap2=NULL;
	int lap_length;
	float * f_storage=NULL;
	float ** coords=NULL;
    int orig_n=n;

	//double conj_tol=tolerance_cg;        /* tolerance of Conjugate Gradient */
	CMajEnvVPSC *cMajEnvHor = NULL;
	CMajEnvVPSC *cMajEnvVrt = NULL;
	clock_t start_time;
	double y_0;
	int length;
	DistType diameter;
	float * Dij=NULL;
	float constant_term;
	int count;
	double degree;
	int step;
	float val;
	double old_stress, new_stress;
	bool converged;
	int len;
    double nsizeScale=0;
    float maxEdgeLen=0;

    initLayout(graph, n, dim, d_coords);
    if (n == 1) return 0;

    for(i=0;i<n;i++) {
        for(j=1;j<graph[i].nedges;j++) {
            maxEdgeLen=MAX(graph[i].ewgts[j],maxEdgeLen);
        }
    }

    fprintf(stderr,"Entered: stress_majorization_vsep, maxEdgeLen=%f\n",maxEdgeLen);

	/****************************************************
	** Compute the all-pairs-shortest-distances matrix **
	****************************************************/

	if (maxi==0)
		return iterations;

    if (model == MODEL_SUBSET) {
        /* weight graph to separate high-degree nodes */
        /* and perform slower Dijkstra-based computation */
        if (Verbose)
            fprintf(stderr, "Calculating subset model");
        Dij = compute_apsp_artifical_weights_packed(graph, n);
    } else if (model == MODEL_CIRCUIT) {
        Dij = circuitModel(graph, n);
        if (!Dij) {
            agerr(AGWARN,
                  "graph is disconnected. Hence, the circuit model\n");
            agerr(AGPREV,
                  "is undefined. Reverting to the shortest path model.\n");
        }
    }
    if (!Dij) {
        if (Verbose)
            fprintf(stderr, "Calculating shortest paths ");
        Dij = compute_apsp_packed(graph, n);
    }

	diameter=-1;
	length = n+n*(n-1)/2;
	for (i=0; i<length; i++) {
		if (Dij[i]>diameter) {
			diameter = (int)Dij[i];
		}
	}

    /* for numerical stability, scale down layout		 */
    /* No Jiggling, might conflict with constraints			 */
    fprintf(stderr,"SCALING!!!\n");
    double max=1;		
    for (i=0; i<dim; i++) {	
        for (j=0; j<n; j++) {
            if (fabs(d_coords[i][j])>max) {
                max=fabs(d_coords[i][j]);
            }
        }	
    }
    for (i=0; i<dim; i++) {	
        for (j=0; j<n; j++) {
            d_coords[i][j]*=10/max;
        }	
    }

	/**************************
	** Layout initialization **
	**************************/

	for (i=0; i<dim; i++) {		
		orthog1(n, d_coords[i]);
	}

	/* for the y-coords, don't center them, but translate them so y[0]=0 */
	y_0 = d_coords[1][0];
	for (i=0; i<n; i++) {
		d_coords[1][i] -= y_0;
	}

	/**************************
	** Laplacian computation **
	**************************/
			
    lap2 = Dij;
    lap_length = n+n*(n-1)/2;
    square_vec(lap_length, lap2);
    /* compute off-diagonal entries */
    invert_vec(lap_length, lap2);
    
    if(clusters->nclusters>0) {
        int nn = n+clusters->nclusters*2;
        fprintf(stderr,"computing clap... n=%d,nn=%d\n",n,nn);
        int clap_length = nn+nn*(nn-1)/2;
        float *clap = N_GNEW(clap_length, float);
        float *cdegrees = N_GNEW(nn, float);
        int c0,c1;
        float v;
        c0=c1=0;
        for(i=0;i<nn;i++) {
            for(j=0;j<nn-i;j++) {
                if(i<n && j<n-i) {
                    v=lap2[c0++];
                } else {
                    //v=j==1?i%2:0;
                    if(j==1&&i%2==1) {
                        v=maxEdgeLen;
                        v*=v;
                        if(v>0.01) {
                            v=1.0/v;
                        }
                    }
                    else v=0;
                }
                //fprintf(stderr," %f",v);
                clap[c1++]=v;
            }
            //fprintf(stderr,"\n");
        }
        free(lap2);
        lap2=clap;
        n=nn;
        lap_length=clap_length;
    }
    /* compute diagonal entries */
    count=0;
    degrees = N_GNEW(n, double);
    set_vector_val(n, 0, degrees);
    for (i=0; i<n-1; i++) {
        degree=0;
        count++; // skip main diag entry
        for (j=1; j<n-i; j++,count++) {
            val = lap2[count];
            degree+=val; degrees[i+j]-=val;
        }
        degrees[i]-=degree;
    }
    for (step=n,count=0,i=0; i<n; i++,count+=step,step--) {
        lap2[count]=(float)degrees[i];
    }

	coords = N_GNEW(dim, float*);
	f_storage = N_GNEW(dim*n, float);
	for (i=0; i<dim; i++) {
		coords[i] = f_storage+i*n;
		for (j=0; j<n; j++) {
			coords[i][j] = j<orig_n?(float)(d_coords[i][j]):0;
		}
	}

	/* compute constant term in stress sum
	 * which is \sum_{i<j} w_{ij}d_{ij}^2
     */
	constant_term=(float)(n*(n-1)/2);
	
	/*************************
	** Layout optimization  **
	*************************/
	
	b = N_GNEW (dim, float*);
	b[0] = N_GNEW (dim*n, float);
	for (k=1; k<dim; k++) {
		b[k] = b[0]+k*n;
	}

	tmp_coords = N_GNEW(n, float);
	dist_accumulator = N_GNEW(n, float);
	
	old_stress=DBL_MAX; /* at least one iteration */

	start_time = clock();

	cMajEnvHor=initCMajVPSC(n,lap2,graph, 0, 0, clusters);
	cMajEnvVrt=initCMajVPSC(n,lap2,graph, diredges, edge_gap, clusters);

	lap1 = N_GNEW(lap_length, float);

    fprintf(stderr,"Entering main loop...\n");
	for (converged=false,iterations=0; iterations<maxi && !converged; iterations++) {

		/* First, construct Laplacian of 1/(d_ij*|p_i-p_j|)  */
		set_vector_val(n, 0, degrees);
		sqrt_vecf(lap_length, lap2, lap1);
		for (count=0,i=0; i<n-1; i++) {
			len = n-i-1;
			/* init 'dist_accumulator' with zeros */
			set_vector_valf(n, 0, dist_accumulator);
			
			/* put into 'dist_accumulator' all squared distances 
             * between 'i' and 'i'+1,...,'n'-1
             */
			for (k=0; k<dim; k++) {	
				set_vector_valf(len, coords[k][i], tmp_coords);
				vectors_mult_additionf(len, tmp_coords, -1, coords[k]+i+1);
				square_vec(len, tmp_coords);
				vectors_additionf(len, tmp_coords, dist_accumulator, dist_accumulator);
			}			
		
			/* convert to 1/d_{ij} */
			invert_sqrt_vec(len, dist_accumulator);
			/* detect overflows */
			for (j=0; j<len; j++) {
				if (dist_accumulator[j]>=FLT_MAX || dist_accumulator[j]<0) {
					dist_accumulator[j]=0;
				}
			}
			
			count++; /* save place for the main diagonal entry */
			degree=0;
			for (j=0; j<len; j++,count++) {
				val=lap1[count]*=dist_accumulator[j];
				degree+=val; degrees[i+j+1]-=val;
			}
			degrees[i]-=degree;			
		}
		for (step=n,count=0,i=0; i<n; i++,count+=step,step--) {
			lap1[count]=(float)degrees[i];
		}

		/* Now compute b[] (L^(X(t))*X(t)) */
		for (k=0; k<dim; k++) {	
			/* b[k] := lap1*coords[k] */
			right_mult_with_vector_ff(lap1, n, coords[k], b[k]);
		}
		
		/* compute new stress
		 * remember that the Laplacians are negated, so we subtract 
         * instead of add and vice versa
         */
		new_stress=0;
		for (k=0; k<dim; k++) {	
			new_stress+=vectors_inner_productf(n, coords[k], b[k]);
		}
		new_stress*=2;
		new_stress+=constant_term; // only after mult by 2		
		for (k=0; k<dim; k++) {	
			right_mult_with_vector_ff(lap2, n, coords[k], tmp_coords);
			new_stress-=vectors_inner_productf(n, coords[k], tmp_coords);
		}

#ifdef ALTERNATIVE_STRESS_CALC
		{
		double mat_stress=new_stress;
		double compute_stress(float ** coords, float * lap, int dim, int n);
		new_stress = compute_stress(coords, lap2, dim, n);
		if (fabs(mat_stress-new_stress)/min(mat_stress,new_stress)>0.001) {
			fprintf(stderr,"Diff stress vals: %lf %lf (iteration #%d)\n", mat_stress, new_stress,iterations);
		}
		}
#endif
		/* check for convergence */
        fprintf(stderr,"stress=%f\n",new_stress);
		converged = fabs(new_stress-old_stress)/fabs(old_stress+1e-10) < Epsilon;
		//converged = converged || (iterations>1 && new_stress>old_stress); 
			/* in first iteration we allowed stress increase, which 
             * might result ny imposing constraints
             */
		old_stress = new_stress;

        // in determining non-overlap constraints we gradually scale up the
        // size of nodes to avoid local minima
        if((iterations>=maxi-1||converged)&&noverlap==1&&nsizeScale<0.999) {
            nsizeScale+=0.01;
            fprintf(stderr,"nsizescale=%f,iterations=%d\n",nsizeScale,iterations);
            iterations=0;
            converged = false;
        }
		

        /* now we find the optimizer of trace(X'LX)+X'B by solving 'dim' 
         * system of equations, thereby obtaining the new coordinates.
         * If we use the constraints (given by the var's: 'ordering', 
         * 'levels' and 'num_levels'), we cannot optimize 
         * trace(X'LX)+X'B by simply solving equations, but we have
         * to use a quadratic programming solver
         * note: 'lap2' is a packed symmetric matrix, that is its 
         * upper-triangular part is arranged in a vector row-wise
         * also note: 'lap2' is really the negated laplacian (the 
         * laplacian is -'lap2')
         */
        
        if(noverlap==1 && nsizeScale > 0.001) {
            generateNonoverlapConstraints(cMajEnvHor,nsize,gap,nsizeScale,coords,0,clusters,nsizeScale<0.5?false:true);
        }
        if(cMajEnvHor->m > 0) {
            constrained_majorization_vpsc(cMajEnvHor, b[0], coords[0], localConstrMajorIterations);
        } else {
            // if there are no constraints then use conjugate gradient
            // optimisation which should be considerably faster
			conjugate_gradient_mkernel(lap2, coords[0], b[0], n, tolerance_cg, n);	
        }
        if(noverlap==1 && nsizeScale > 0.001) {
            generateNonoverlapConstraints(cMajEnvVrt,nsize,gap,nsizeScale,coords,1,clusters,false);
        }
        if(cMajEnvVrt->m > 0) {
            constrained_majorization_vpsc(cMajEnvVrt, b[1], coords[1], localConstrMajorIterations);
        } else {
			conjugate_gradient_mkernel(lap2, coords[1], b[1], n, tolerance_cg, n);	
        }
	}
    fprintf(stderr,"Finished majorization!\n");
	deleteCMajEnvVPSC(cMajEnvHor);
	deleteCMajEnvVPSC(cMajEnvVrt);
    fprintf(stderr,"  freed cMajEnv!\n");

    if (noverlap==2) {
        fprintf(stderr,"Removing overlaps as post-process...\n");
        removeoverlaps(orig_n,coords,nsize,gap,clusters);
    }
	
	if (coords!=NULL) {
		for (i=0; i<dim; i++) {
			for (j=0; j<orig_n; j++) {
				d_coords[i][j] = coords[i][j];
			}
		}
		free (coords[0]); free (coords);
	}
	
	if (b) {	
		free (b[0]); free (b);
	}
	free (tmp_coords);
	free (dist_accumulator);
	free (degrees);
	free (lap2);
	free (lap1); 

    return iterations;
}
#endif /* DIGCOLA */

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4 :
