#!/usr/bin/env python3
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import sys

def simulate_1st_order(tau, u_cmd, dt, y0=0.0):
    """Simula a dinâmica y_dot = (u_cmd - y) / tau usando integração de Euler."""
    y_sim = np.zeros_like(u_cmd)
    y_sim[0] = y0
    for i in range(1, len(u_cmd)):
        y_sim[i] = y_sim[i-1] + (dt / tau) * (u_cmd[i-1] - y_sim[i-1])
    return y_sim

def loss_function(tau, t, u_cmd, y_meas, dt):
    """Função de custo: erro quadrático médio."""
    if tau <= 0.0:
        return 1e6  # Penalização física (tau deve ser estritamente positivo)
    y_sim = simulate_1st_order(tau, u_cmd, dt, y0=y_meas[0])
    return np.mean((y_meas - y_sim) ** 2)

def run_identification(time_arr, cmd_arr, meas_arr, angle_name="Roll"):
    dt = time_arr[1] - time_arr[0]
    
    # Chute inicial: tau = 0.25 segundos
    initial_guess = [0.25]
    
    res = minimize(loss_function, initial_guess, args=(time_arr, cmd_arr, meas_arr, dt), method='Nelder-Mead')
    estimated_tau = res.x[0]
    
    print(f"--- Identificação do {angle_name} ---")
    print(f"Constante de tempo estimada (tau): {estimated_tau:.4f} s")
    print(f"Largura de Banda equivalente (1/tau): {1.0/estimated_tau:.2f} rad/s")
    
    # Plot para validação visual
    y_sim = simulate_1st_order(estimated_tau, cmd_arr, dt, y0=meas_arr[0])
    
    plt.figure(figsize=(10, 5))
    plt.plot(time_arr, cmd_arr, '--', label=f'Comando ({angle_name}_cmd)', color='gray')
    plt.plot(time_arr, meas_arr, label=f'Medido ({angle_name}_meas)', color='blue', alpha=0.7)
    plt.plot(time_arr, y_sim, label=f'Modelo Simulado (tau={estimated_tau:.3f}s)', color='red', linewidth=2)
    plt.title(f"Identificação da Dinâmica de Atitude: {angle_name}")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Ângulo (rad)")
    plt.legend()
    plt.grid(True)
    
    output_png = f"sysid_{angle_name.lower()}_result.png"
    plt.savefig(output_png)
    print(f"Gráfico de validação guardado em: {output_png}")
    plt.show()
    
    return estimated_tau

if __name__ == "__main__":
    print("Para correr a identificação com dados reais, importa os teus dados de ROSBag e chama a função 'run_identification'.")
    print("A gerar dados de domínio (sintéticos) com tau = 0.18s...")
    
    t = np.linspace(0, 10, 1000)
    dt = t[1] - t[0]
    
    # Comando doublet
    cmd = np.zeros_like(t)
    cmd[(t > 1) & (t < 3)] = 0.2
    cmd[(t > 3) & (t < 5)] = -0.2
    cmd[(t > 6) & (t < 8)] = 0.15
    
    # Resposta real do drone simulada com tau=0.18s e ruído
    tau_real = 0.18
    meas = np.zeros_like(t)
    for i in range(1, len(t)):
        meas[i] = meas[i-1] + (dt / tau_real) * (cmd[i-1] - meas[i-1])
    meas += np.random.normal(0, 0.003, size=t.shape)
    
    run_identification(t, cmd, meas, "Roll")
