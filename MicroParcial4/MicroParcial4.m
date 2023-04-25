%{
    Los argumentos de la función son: Wn,Zeta,tr,ts,Mp,figuras
    
    donde Wn y Zeta es para la función de transferencia de la forma
    Wn^2/(s^2 + 2*Zeta*Wn*s + Wn^2)

    tr = Rise time en segundos
    ts = Settlins time en segundos
    Mp = Oversoot in %

    figuras = 1 si se quiere mostrar la respuesta del sistema original y de
    sistema controlado, sino figuras = 0
%}
  
[Kp,Ki,Kd] = SolverPID(1,2,0.1,0.5,10,1);