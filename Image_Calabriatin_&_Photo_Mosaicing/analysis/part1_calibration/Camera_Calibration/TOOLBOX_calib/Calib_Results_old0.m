% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1089.020928288319283 ; 1082.861667073299259 ];

%-- Principal point:
cc = [ 487.234961978965316 ; 655.313088996842453 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.007365155384878 ; -0.104119031139584 ; -0.000880845347284 ; 0.000431211691374 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.364973556877414 ; 1.446830865058681 ];

%-- Principal point uncertainty:
cc_error = [ 1.953543998775681 ; 2.337316037393160 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006843746138473 ; 0.024069023269858 ; 0.000716936707419 ; 0.000603737771462 ; 0.000000000000000 ];

%-- Image size:
nx = 977;
ny = 1303;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 25;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.589932e+00 ; 1.591356e+00 ; -9.375761e-01 ];
Tc_1  = [ -5.878013e+01 ; -6.159317e+00 ; 3.421092e+02 ];
omc_error_1 = [ 1.889037e-03 ; 2.102438e-03 ; 2.792013e-03 ];
Tc_error_1  = [ 6.133854e-01 ; 7.475689e-01 ; 3.852860e-01 ];

%-- Image #2:
omc_2 = [ 1.918697e+00 ; 1.912959e+00 ; -5.778233e-01 ];
Tc_2  = [ -5.983559e+01 ; -5.083296e+01 ; 2.845105e+02 ];
omc_error_2 = [ 2.048044e-03 ; 2.094903e-03 ; 3.793930e-03 ];
Tc_error_2  = [ 5.088881e-01 ; 6.205879e-01 ; 3.824038e-01 ];

%-- Image #3:
omc_3 = [ 1.952216e+00 ; 2.301347e+00 ; -3.341401e-01 ];
Tc_3  = [ -5.072666e+01 ; -8.387797e+01 ; 2.620746e+02 ];
omc_error_3 = [ 2.062192e-03 ; 2.504349e-03 ; 4.985192e-03 ];
Tc_error_3  = [ 4.747963e-01 ; 5.697440e-01 ; 4.437816e-01 ];

%-- Image #4:
omc_4 = [ -2.170544e+00 ; -1.851405e+00 ; 9.628958e-01 ];
Tc_4  = [ -7.216737e+01 ; -6.585829e+01 ; 2.756825e+02 ];
omc_error_4 = [ 2.271010e-03 ; 1.403789e-03 ; 3.459672e-03 ];
Tc_error_4  = [ 5.035232e-01 ; 6.040659e-01 ; 3.607192e-01 ];

%-- Image #5:
omc_5 = [ 1.217796e+00 ; 1.845909e+00 ; -1.743395e-01 ];
Tc_5  = [ -2.166934e+01 ; -8.302858e+01 ; 3.211305e+02 ];
omc_error_5 = [ 1.896077e-03 ; 1.925542e-03 ; 2.876664e-03 ];
Tc_error_5  = [ 5.808453e-01 ; 6.983533e-01 ; 4.686608e-01 ];

%-- Image #6:
omc_6 = [ -1.781359e+00 ; -1.920484e+00 ; -5.243167e-01 ];
Tc_6  = [ -5.846952e+01 ; -2.874842e+01 ; 2.156795e+02 ];
omc_error_6 = [ 1.793697e-03 ; 2.161649e-03 ; 3.327301e-03 ];
Tc_error_6  = [ 3.891431e-01 ; 4.787409e-01 ; 4.247140e-01 ];

%-- Image #7:
omc_7 = [ 2.066565e+00 ; 2.098861e+00 ; 4.831513e-01 ];
Tc_7  = [ -6.092979e+01 ; -8.493924e+01 ; 2.167833e+02 ];
omc_error_7 = [ 2.319117e-03 ; 2.200309e-03 ; 4.651466e-03 ];
Tc_error_7  = [ 4.155227e-01 ; 4.900131e-01 ; 4.840609e-01 ];

%-- Image #8:
omc_8 = [ 2.213676e+00 ; 2.104192e+00 ; 4.108410e-01 ];
Tc_8  = [ -3.423716e+01 ; -4.761243e+01 ; 2.319946e+02 ];
omc_error_8 = [ 2.630248e-03 ; 1.897040e-03 ; 4.786696e-03 ];
Tc_error_8  = [ 4.256685e-01 ; 5.096886e-01 ; 4.932902e-01 ];

%-- Image #9:
omc_9 = [ -1.675702e+00 ; -2.029993e+00 ; -5.789222e-02 ];
Tc_9  = [ -3.900338e+01 ; -1.051747e+02 ; 2.690951e+02 ];
omc_error_9 = [ 2.413790e-03 ; 2.370086e-03 ; 4.193319e-03 ];
Tc_error_9  = [ 4.997882e-01 ; 6.020881e-01 ; 5.344435e-01 ];

%-- Image #10:
omc_10 = [ -1.895615e+00 ; -2.095811e+00 ; -2.225101e-01 ];
Tc_10  = [ -6.469768e+01 ; -7.352890e+01 ; 1.941365e+02 ];
omc_error_10 = [ 1.951966e-03 ; 2.004029e-03 ; 3.748870e-03 ];
Tc_error_10  = [ 3.584891e-01 ; 4.403706e-01 ; 3.862495e-01 ];

%-- Image #11:
omc_11 = [ -1.853966e+00 ; -2.144244e+00 ; -5.660490e-01 ];
Tc_11  = [ -5.608027e+01 ; -6.613946e+01 ; 1.816159e+02 ];
omc_error_11 = [ 1.705753e-03 ; 2.139596e-03 ; 3.717424e-03 ];
Tc_error_11  = [ 3.391050e-01 ; 4.199410e-01 ; 3.849135e-01 ];

%-- Image #12:
omc_12 = [ -1.985128e+00 ; -1.523849e+00 ; 1.137403e-02 ];
Tc_12  = [ -7.715148e+01 ; -4.431819e+01 ; 2.327138e+02 ];
omc_error_12 = [ 2.156510e-03 ; 1.640888e-03 ; 2.983499e-03 ];
Tc_error_12  = [ 4.208580e-01 ; 5.102024e-01 ; 3.888384e-01 ];

%-- Image #13:
omc_13 = [ -1.941817e+00 ; -1.996759e+00 ; 1.361646e+00 ];
Tc_13  = [ -3.211095e+01 ; -6.634903e+01 ; 2.745312e+02 ];
omc_error_13 = [ 2.224978e-03 ; 1.402371e-03 ; 3.310625e-03 ];
Tc_error_13  = [ 5.006900e-01 ; 5.989569e-01 ; 3.010589e-01 ];

%-- Image #14:
omc_14 = [ 1.633391e+00 ; 1.427064e+00 ; -1.619310e-01 ];
Tc_14  = [ -8.047964e+01 ; -6.135939e+01 ; 2.399964e+02 ];
omc_error_14 = [ 2.002890e-03 ; 1.643012e-03 ; 2.495431e-03 ];
Tc_error_14  = [ 4.339604e-01 ; 5.290787e-01 ; 3.702819e-01 ];

%-- Image #15:
omc_15 = [ 1.574758e+00 ; 1.830599e+00 ; -8.107371e-02 ];
Tc_15  = [ -3.758276e+01 ; -8.030657e+01 ; 2.333944e+02 ];
omc_error_15 = [ 1.902974e-03 ; 1.679138e-03 ; 2.888273e-03 ];
Tc_error_15  = [ 4.215817e-01 ; 5.065451e-01 ; 3.588530e-01 ];

%-- Image #16:
omc_16 = [ -1.448284e+00 ; -2.004336e+00 ; -1.145161e+00 ];
Tc_16  = [ -4.018513e+01 ; -4.210856e+01 ; 1.676833e+02 ];
omc_error_16 = [ 1.562711e-03 ; 2.338470e-03 ; 2.932606e-03 ];
Tc_error_16  = [ 3.102046e-01 ; 3.846581e-01 ; 3.759150e-01 ];

%-- Image #17:
omc_17 = [ -1.554463e+00 ; -1.533708e+00 ; -3.464261e-01 ];
Tc_17  = [ -6.166105e+01 ; -2.653873e+01 ; 1.776578e+02 ];
omc_error_17 = [ 1.784163e-03 ; 1.853392e-03 ; 2.396079e-03 ];
Tc_error_17  = [ 3.189486e-01 ; 3.903112e-01 ; 3.022115e-01 ];

%-- Image #18:
omc_18 = [ -1.637057e+00 ; -1.618682e+00 ; -4.050537e-01 ];
Tc_18  = [ -5.985204e+01 ; -3.369819e+01 ; 2.086557e+02 ];
omc_error_18 = [ 1.835399e-03 ; 1.919069e-03 ; 2.676312e-03 ];
Tc_error_18  = [ 3.762284e-01 ; 4.599398e-01 ; 3.766386e-01 ];

%-- Image #19:
omc_19 = [ -1.974474e+00 ; -1.762490e+00 ; 3.166655e-01 ];
Tc_19  = [ -6.500356e+01 ; -6.647996e+01 ; 2.363305e+02 ];
omc_error_19 = [ 2.127843e-03 ; 1.616215e-03 ; 3.263870e-03 ];
Tc_error_19  = [ 4.290544e-01 ; 5.145497e-01 ; 3.763006e-01 ];

%-- Image #20:
omc_20 = [ -2.204041e+00 ; -2.084861e+00 ; -1.706800e-01 ];
Tc_20  = [ -6.101025e+01 ; -5.958527e+01 ; 1.871952e+02 ];
omc_error_20 = [ 1.940998e-03 ; 2.029612e-03 ; 4.094299e-03 ];
Tc_error_20  = [ 3.463807e-01 ; 4.186671e-01 ; 3.688948e-01 ];

%-- Image #21:
omc_21 = [ -2.147611e+00 ; -1.878648e+00 ; 7.250251e-01 ];
Tc_21  = [ -7.706977e+01 ; -6.942752e+01 ; 2.464088e+02 ];
omc_error_21 = [ 2.174573e-03 ; 1.381739e-03 ; 3.441418e-03 ];
Tc_error_21  = [ 4.493616e-01 ; 5.404368e-01 ; 3.583291e-01 ];

%-- Image #22:
omc_22 = [ -2.146819e+00 ; -2.249078e+00 ; -4.186525e-02 ];
Tc_22  = [ -5.887128e+01 ; -7.409417e+01 ; 1.941337e+02 ];
omc_error_22 = [ 1.942186e-03 ; 2.072366e-03 ; 4.313110e-03 ];
Tc_error_22  = [ 3.596457e-01 ; 4.320121e-01 ; 3.810579e-01 ];

%-- Image #23:
omc_23 = [ 2.111126e+00 ; 2.138344e+00 ; 4.278125e-01 ];
Tc_23  = [ -4.930510e+01 ; -6.678285e+01 ; 1.782592e+02 ];
omc_error_23 = [ 2.106401e-03 ; 1.694089e-03 ; 3.978414e-03 ];
Tc_error_23  = [ 3.361593e-01 ; 4.009324e-01 ; 3.714405e-01 ];

%-- Image #24:
omc_24 = [ -1.876967e+00 ; -1.972626e+00 ; -1.291852e-01 ];
Tc_24  = [ -5.533478e+01 ; -7.907141e+01 ; 2.295865e+02 ];
omc_error_24 = [ 2.195835e-03 ; 2.110370e-03 ; 3.884265e-03 ];
Tc_error_24  = [ 4.219113e-01 ; 5.142092e-01 ; 4.401954e-01 ];

%-- Image #25:
omc_25 = [ -1.690910e+00 ; -2.070207e+00 ; -9.611560e-01 ];
Tc_25  = [ -4.680822e+01 ; -3.473286e+01 ; 1.694048e+02 ];
omc_error_25 = [ 1.537186e-03 ; 2.306450e-03 ; 3.193046e-03 ];
Tc_error_25  = [ 3.104076e-01 ; 3.848621e-01 ; 3.663069e-01 ];

