% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1090.048906273493913 ; 1083.351308611594504 ];

%-- Principal point:
cc = [ 487.744353444881654 ; 656.655963976008479 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.010157350072254 ; -0.112375146964025 ; -0.000780008050982 ; 0.000577805648985 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.438672040982105 ; 1.575583801216932 ];

%-- Principal point uncertainty:
cc_error = [ 2.298294488755491 ; 2.687853101407646 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.007796248078589 ; 0.028033855646182 ; 0.000827956106495 ; 0.000716644762899 ; 0.000000000000000 ];

%-- Image size:
nx = 977;
ny = 1303;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.591142e+00 ; 1.592216e+00 ; -9.360520e-01 ];
Tc_1  = [ -5.893792e+01 ; -6.591876e+00 ; 3.422699e+02 ];
omc_error_1 = [ 2.070475e-03 ; 2.319219e-03 ; 3.107335e-03 ];
Tc_error_1  = [ 7.197772e-01 ; 8.573950e-01 ; 4.092762e-01 ];

%-- Image #2:
omc_2 = [ 1.919616e+00 ; 1.913696e+00 ; -5.760490e-01 ];
Tc_2  = [ -5.996506e+01 ; -5.118424e+01 ; 2.845444e+02 ];
omc_error_2 = [ 2.165814e-03 ; 2.270396e-03 ; 4.095644e-03 ];
Tc_error_2  = [ 5.974419e-01 ; 7.120196e-01 ; 4.170656e-01 ];

%-- Image #3:
omc_3 = [ 1.952683e+00 ; 2.301836e+00 ; -3.329120e-01 ];
Tc_3  = [ -5.084504e+01 ; -8.418690e+01 ; 2.621503e+02 ];
omc_error_3 = [ 2.133710e-03 ; 2.647140e-03 ; 5.240062e-03 ];
Tc_error_3  = [ 5.574570e-01 ; 6.546864e-01 ; 4.814476e-01 ];

%-- Image #4:
omc_4 = [ -2.170207e+00 ; -1.851797e+00 ; 9.605594e-01 ];
Tc_4  = [ -7.231115e+01 ; -6.621287e+01 ; 2.756389e+02 ];
omc_error_4 = [ 2.529609e-03 ; 1.496736e-03 ; 3.902097e-03 ];
Tc_error_4  = [ 5.901454e-01 ; 6.936632e-01 ; 4.027302e-01 ];

%-- Image #5:
omc_5 = [ 1.218436e+00 ; 1.845738e+00 ; -1.724869e-01 ];
Tc_5  = [ -2.181982e+01 ; -8.342949e+01 ; 3.211723e+02 ];
omc_error_5 = [ 2.070073e-03 ; 2.136086e-03 ; 3.217033e-03 ];
Tc_error_5  = [ 6.807796e-01 ; 8.019200e-01 ; 5.075585e-01 ];

%-- Image #6:
omc_6 = [ -1.780776e+00 ; -1.920301e+00 ; -5.253606e-01 ];
Tc_6  = [ -5.856862e+01 ; -2.902549e+01 ; 2.158653e+02 ];
omc_error_6 = [ 1.941682e-03 ; 2.378159e-03 ; 3.705653e-03 ];
Tc_error_6  = [ 4.559920e-01 ; 5.492504e-01 ; 4.439361e-01 ];

%-- Image #7:
omc_7 = [ 2.066252e+00 ; 2.098175e+00 ; 4.846697e-01 ];
Tc_7  = [ -6.101947e+01 ; -8.522179e+01 ; 2.168693e+02 ];
omc_error_7 = [ 2.441510e-03 ; 2.270108e-03 ; 4.935600e-03 ];
Tc_error_7  = [ 4.848026e-01 ; 5.611219e-01 ; 5.197729e-01 ];

%-- Image #8:
omc_8 = [ 2.213235e+00 ; 2.103621e+00 ; 4.125517e-01 ];
Tc_8  = [ -3.434088e+01 ; -4.791049e+01 ; 2.320945e+02 ];
omc_error_8 = [ 2.807774e-03 ; 1.969649e-03 ; 5.153091e-03 ];
Tc_error_8  = [ 4.978901e-01 ; 5.842236e-01 ; 5.161996e-01 ];

%-- Image #9:
omc_9 = [ -1.675075e+00 ; -2.029823e+00 ; -5.845673e-02 ];
Tc_9  = [ -3.912718e+01 ; -1.055271e+02 ; 2.692774e+02 ];
omc_error_9 = [ 2.507624e-03 ; 2.474235e-03 ; 4.436296e-03 ];
Tc_error_9  = [ 5.863145e-01 ; 6.900991e-01 ; 5.638633e-01 ];

%-- Image #10:
omc_10 = [ -1.895404e+00 ; -2.095926e+00 ; -2.226805e-01 ];
Tc_10  = [ -6.478643e+01 ; -7.379813e+01 ; 1.943066e+02 ];
omc_error_10 = [ 2.069549e-03 ; 2.155414e-03 ; 4.103820e-03 ];
Tc_error_10  = [ 4.207066e-01 ; 5.050876e-01 ; 4.153625e-01 ];

%-- Image #11:
omc_11 = [ -1.853937e+00 ; -2.144402e+00 ; -5.665140e-01 ];
Tc_11  = [ -5.616844e+01 ; -6.640731e+01 ; 1.817952e+02 ];
omc_error_11 = [ 1.815365e-03 ; 2.333856e-03 ; 4.061754e-03 ];
Tc_error_11  = [ 3.967780e-01 ; 4.808297e-01 ; 4.088919e-01 ];

%-- Image #12:
omc_12 = [ -1.984352e+00 ; -1.523829e+00 ; 1.066521e-02 ];
Tc_12  = [ -7.725440e+01 ; -4.461376e+01 ; 2.328557e+02 ];
omc_error_12 = [ 2.333645e-03 ; 1.798559e-03 ; 3.335610e-03 ];
Tc_error_12  = [ 4.944800e-01 ; 5.856945e-01 ; 4.153900e-01 ];

%-- Image #13:
omc_13 = [ -1.941453e+00 ; -1.997419e+00 ; 1.358591e+00 ];
Tc_13  = [ -3.224307e+01 ; -6.676772e+01 ; 2.744837e+02 ];
omc_error_13 = [ 2.524658e-03 ; 1.524024e-03 ; 3.784799e-03 ];
Tc_error_13  = [ 5.874591e-01 ; 6.877041e-01 ; 3.350822e-01 ];

%-- Image #14:
omc_14 = [ 1.634235e+00 ; 1.427234e+00 ; -1.603309e-01 ];
Tc_14  = [ -8.057165e+01 ; -6.166658e+01 ; 2.399929e+02 ];
omc_error_14 = [ 2.212764e-03 ; 1.837695e-03 ; 2.848789e-03 ];
Tc_error_14  = [ 5.091336e-01 ; 6.073671e-01 ; 4.108596e-01 ];

%-- Image #15:
omc_15 = [ 1.575360e+00 ; 1.830419e+00 ; -7.887485e-02 ];
Tc_15  = [ -3.769268e+01 ; -8.058956e+01 ; 2.333382e+02 ];
omc_error_15 = [ 2.083009e-03 ; 1.861200e-03 ; 3.276075e-03 ];
Tc_error_15  = [ 4.944618e-01 ; 5.819384e-01 ; 3.990079e-01 ];

%-- Image #16:
omc_16 = [ -1.447910e+00 ; -2.004214e+00 ; -1.146156e+00 ];
Tc_16  = [ -4.025875e+01 ; -4.235615e+01 ; 1.678448e+02 ];
omc_error_16 = [ 1.731660e-03 ; 2.596922e-03 ; 3.313489e-03 ];
Tc_error_16  = [ 3.621490e-01 ; 4.397398e-01 ; 3.960968e-01 ];

%-- Image #17:
omc_17 = [ -1.554176e+00 ; -1.533966e+00 ; -3.469149e-01 ];
Tc_17  = [ -6.173797e+01 ; -2.676091e+01 ; 1.777879e+02 ];
omc_error_17 = [ 1.993271e-03 ; 2.090921e-03 ; 2.751307e-03 ];
Tc_error_17  = [ 3.745056e-01 ; 4.477317e-01 ; 3.238492e-01 ];

%-- Image #18:
omc_18 = [ -1.636532e+00 ; -1.618839e+00 ; -4.056076e-01 ];
Tc_18  = [ -5.994128e+01 ; -3.397522e+01 ; 2.088042e+02 ];
omc_error_18 = [ 2.021093e-03 ; 2.138291e-03 ; 3.025829e-03 ];
Tc_error_18  = [ 4.415903e-01 ; 5.275095e-01 ; 3.975416e-01 ];

%-- Image #19:
omc_19 = [ -1.973636e+00 ; -1.762382e+00 ; 3.161524e-01 ];
Tc_19  = [ -6.509046e+01 ; -6.677114e+01 ; 2.364468e+02 ];
omc_error_19 = [ 2.295708e-03 ; 1.739252e-03 ; 3.613716e-03 ];
Tc_error_19  = [ 5.035495e-01 ; 5.912771e-01 ; 4.079285e-01 ];

%-- Image #20:
omc_20 = [ -2.204144e+00 ; -2.085080e+00 ; -1.712786e-01 ];
Tc_20  = [ -6.109766e+01 ; -5.983533e+01 ; 1.873462e+02 ];
omc_error_20 = [ 2.058105e-03 ; 2.178125e-03 ; 4.440654e-03 ];
Tc_error_20  = [ 4.061987e-01 ; 4.805202e-01 ; 3.934249e-01 ];

