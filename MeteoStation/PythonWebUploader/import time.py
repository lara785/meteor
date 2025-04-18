import time

class Gear:
    def __init__(self, delay):
        self.delay = delay
        self.position = 0

    def step(self):
        self.position += 1
        time.sleep(self.delay)

# Criar as engrenagens
main_gear = Gear(0.001)  # 1ms delay
controlled_gear = Gear(0.005)  # 5ms delay

# Simular a rotação
for _ in range(100):  # Simular 100 passos
    main_gear.step()
    controlled_gear.step()

print("Posição da engrenagem principal:", main_gear.position)
print("Posição da engrenagem controlada:", controlled_gear.position)
