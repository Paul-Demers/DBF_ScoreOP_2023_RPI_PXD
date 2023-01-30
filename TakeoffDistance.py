from math import pi, sqrt
from scipy.integrate import solve_ivp, cumtrapz

g = 32.2  # Acceleration due to gravity in ft/s^2
rho = .00222*g  # Air density at sea level in slug/ft^3 to lb/ft3

#%% Equation - Drag

def calc_drag(W, b, AR, l_f, u, Takeoff=False, h_o=7, cL_max=1.5):
    """ calc the drag
    INPUTS: Weight [lb], wingspan [ft], aspect ratio, velocity [fps]
            GE - Ground Effect; only relevant for takeoff
            h_o - wing distance from ground [in]
    """
    S = b**2/AR
    cL = W/((1/2)*rho*(u+0.001)**2*S) # cruise: L = W # u+0.01 to eliminate divide by 0 error
    e = 1.78*(1-0.045*AR**0.68) - 0.64 # Oswald Efficiency Factor for Complete Airplane - Anderson eq. 6.25
    #e = 0.8 # typically 0.6 for low-wing, 0.8 for high-wing - McCormick pg 175
    
    cD_parasite = 0.008/S + 0.0088 + rho/(S*l_f**2) + .0006336*b/l_f + .008*b/l_f
    
    cD_induced = cL**2/(pi*AR*e)
    
    if Takeoff==True:  
        x_GE = 16*((h_o/12)/b)**2
        cD_IGE_OGE = x_GE/(1+x_GE) # Drag coefficient ground effect ratio; only affects induced drag
        cD_induced = cL_max**2/(pi*AR*e) * cD_IGE_OGE
    
    cD = cD_parasite + cD_induced
    return cD

#%% Equation - Takeoff Distance

def takeoff_distance(W, b, AR, l_f, C_T, h_o=7, v0=0, cL_max=1.5, t_end=30, max_step=0.1):
    """ determine if thrust is sufficient for takeoff in x distance
    INPUTS: W - weight [lb]
            b - wingspan [ft]
            AR - aspect ratio []
            l_f - fuselage length [ft]
            C_T - static thrust coefficient; thrust at 0 velocity [lb] **PHYSICALLY MEASURE THIS
            v0 - initial takeoff velocity; ground launch vs. hand launch [fps]
            t_end - range to search for takeoff velocity [s]
            max_step - refinement
    OUTPUT: takeoff_distance - self explanatory [ft]
    CAN OUTPUT: takeoff time, takeoff velocity, plots
    """
    # Define constants
    Sw = (b**2) / AR  # Wing area in ft^2
    f_to = Sw  # Flat Plate skin friction area in ft^2
    mu = .04  # Friction coefficient
    
    C_T_ex = 248800 # example values from Fall 2022 Flight Mech Project - Boeing 767-300 ER
    A_T_ex = 0.386/C_T_ex
    B_T_ex = -672/C_T_ex
    
    A_T = C_T * A_T_ex  # ~= 0; Thrust coefficient A in N*s^2/m^2
    B_T = C_T * B_T_ex # ~= 0; Thrust coefficient B in N*s/m
    
    #%% Calculate useful parameters   
    V_s = sqrt(W / ((1/2)*rho * Sw * cL_max)) # stall speed with flaps out
    V_lof = 1.1*V_s  # Lift off velocity in ft/s
    T_min = mu*W + V_lof**2*W/(2*(2/3)*60*g) # minimum thrust [lb] required to take off at (2/3) of 60 [ft]
    
    cD = calc_drag(W, b, AR, l_f, 0, Takeoff=True, h_o=h_o)

    #%% Define coefficients for differential equation using the input parameters
    A = A_T - (1/2)*rho*Sw*cD + mu*(1/2)*rho*Sw*cL_max
    B = B_T
    C = C_T - mu*W
    
    #%% defining function to be solved
    def dVdt(t, v):
        dVdt = (A*v**2 + B*v + C)*(g/W)
        return dVdt
    
    #%% Alternative function; same results, written for disection and understanding
    def dVdt2(t, v):
        q = (1/2)*rho*v**2
        
        L = q*Sw*cL_max
        D = q*Sw*cD
        T = A_T*v**2 + B_T*v + C_T
        
        vprime = T - D - mu*(W-L)
        vprime *= g/W
        return(vprime)
    
    #%% solving IVP using runge kutta 45
    y0 = [v0]  # initial condition y(t=0)
    t_span = [0, t_end]

    sol = solve_ivp(dVdt, t_span, y0, method='RK45', max_step=max_step)

    # checks to make sure IVP solved properly
    if not (sol.success == True):
        raise Warning(f'Error Solving Thrust Velocity IVP: {sol.message}')

    # output time and velocity arrays
    t = sol.t
    V = sol.y[0]

    #%% integrating velocity to get distance using trapezoidal method
    x = cumtrapz(V, t, initial=0)

    # finding the distance when takeoff velocity is achieved
    takeoff_distance = -1
    for i in range(t.size):
        if V[i] > V_lof:
            takeoff_distance = x[i - 1]
            takeoff_time = t[i - 1]
            t = t[:i]
            V = V[:i]
            x = x[:i]
            break

    # raises an error if it didn't take off in time
    if takeoff_distance == -1:
        raise ValueError(f'\nTakeoff Velocity {round(V_lof*100)/100} [fps] was not reached within t_end = {t_end} [s]'
                         + f'\nMinimum Thrust required: {round(T_min*100)/100} [lb]')
                         
            

    # OTHER DEFINED OUTPUT VARIABLES IF NEEDED IN THE FUTURE
    #    takeoff_time = time of takeoff
    #    t = 1d time array from 0 to takeoff time
    #    V = 1d velocity array (ending at takeoff time)
    #    x = 1d position array (ending at takeoff time)

#%% Sanity Check & return statement

    if __name__ == "__main__":
        import matplotlib.pyplot as plt
        plt.plot(x, V, 'ro-')
        plt.xlabel('distance [ft]')
        plt.ylabel('velocity [fps]')
        plt.grid()
        plt.show()
        
        print('liftoff distance:\t', round(takeoff_distance*100)/100, '[ft]')
        print('liftoff velocity:\t', round(V_lof*100)/100, '[fps]')
        print("minimum thrust required:\t", round(T_min*100)/100, '[lb]')
        print('drag coefficient:\t', round(cD*10000)/10000)

    return takeoff_distance

if __name__ == "__main__":
    W = 11 # lb
    b = 3.5 # ft
    AR = 6.34 
    l_f = 4 # ft
    C_T = 1.5 # lb
    takeoff_distance(W, b, AR, l_f, C_T)