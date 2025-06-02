function MPC_sfunction(block)
%   starting point

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================

function setup(block)

% Register number of ports
block.NumInputPorts  = 5;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1, 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%   Register all relevant methods
block.RegBlockMethod('Outputs', @Outputs);     % Required
%block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

%   Extract x and y postions from input data
v0 = block.InputPort(1).Data;
v_ref = block.InputPort(2).Data;
a0 = block.InputPort(3).Data;
theta = block.InputPort(4).Data;
u_0 = block.InputPort(5).Data;

mu = 0.7; %Crr = 0.01;
m = 2500;
g =9.8;
rho = 1.225;
Cd = 0.28;
Aref = 2.5;  
N = 5;
T_MPC = 0.01;
u0 = u_0 * ones(N, 1);
%persistent u_prev
%if isempty(u_prev)
%    u_prev = u_0 * ones(N,1);
%end
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp','MaxFunctionEvaluations',10000);
%   Use the top line when imposing a hard constraint on "mountain
%   avoidance" - use the bottom line for a soft constraint (penalty)
u_opt = fmincon(@(u)objective(u,v0, v_ref,N,T_MPC,theta),u0,[],[],[],[],[],[],@(u)constraint(u,v0,N,T_MPC,theta),options);
% u_opt = fmincon(@(u)objective(u,x_pos,y_pos,N,T_MPC,theta),u0,[],[],[],[],[],[],[],options);
%   MPC output is the first step of the optimized control trajectory
%u_prev = u_opt;
block.OutputPort(1).Data = u_opt(1);

%end Outputs


%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate
