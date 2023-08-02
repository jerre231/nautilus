'''
Implemente um modelo que descreva os AUVs da UFRJ Nautilus

Requerimentos

Deve conter os atributos: número de thursters, lista com sensores, ano de
construção, nome do veículo, e no mínimo, mais 1 atributo de livre escolha
Deve conter métodos para:
◦ Exibir todos os AUVs em tabela 
◦ Exibir os robôs individulmente
◦ Rankear os robôs do mais novo para o mais antigo
Deve conter outro método de livre escolha
'''

import pandas #biblioteca para criação de tabelas
import os #biblioteca do sistema operacional
import platform as pt #biblioteca utilizada para detectar qual sistema operacional está utilizando

#função utiizada para limpar tela do terminal (utilizada no while)
def clearscreen():
    if pt.system() == 'Windows':
        os.system('cls')
    else:
        os.system('clear')

#definição da classe e dos atributos dos Auv's
class Auv:
    def __init__(self, thrusters, lista_sensores, ano, nome, equipe, tempo_teste):
        self.thrusters = thrusters
        self.sensores = lista_sensores
        self.ano = ano
        self.nome = nome
        self.equipe = equipe
        self.tempo = tempo_teste

    #método de exibição de um dos auv's em uma tabela pandas
    def exibir_auv(self):
        data = {
            'Nome': [self.nome],
            'Thrusters': [self.thrusters],
            'Sensores': [self.sensores],
            'Ano': [self.ano],
            'Equipe': [self.equipe],
            'Tempo de teste': [self.tempo]
        }

        df = pandas.DataFrame(data)
        print(df)
    
    #método de exibição de todos os auv's em uma tabela pandas
    def exibir_todos(lista_auv):
        data = {
            'Nome': [],
            'Thrusters': [],
            'Sensores': [],
            'Ano': [],
            'Equipe': [],
            'Tempo de teste': []
        }

        for auv in lista_auv:
            data['Nome'].append(auv.nome)
            data['Thrusters'].append(auv.thrusters)
            data['Sensores'].append(auv.sensores)
            data['Ano'].append(auv.ano)
            data['Equipe'].append(auv.equipe)
            data['Tempo de teste'].append(auv.tempo)
        
        df = pandas.DataFrame(data)
        print(df)

    #método de ranking por ano
    def rank_ano(lista_auv):
        lista_decrescente = sorted(lista_auv, key=lambda auv: auv.ano, reverse=True)
        print("ranking por ano:")
        for auv in lista_decrescente:
            print(f"{auv.nome} - {auv.ano}")

    #método de ranking por tempo de teste
    def rank_teste(lista_auv):
        lista_crescente = sorted(lista_auv, key=lambda auv: auv.tempo)
        print("ranking por tempo de teste:")
        for auv in lista_crescente:
            print(f"{auv.nome} - {auv.tempo}")



#declaração dos auv's
lua = Auv(8, ['profundidade', 'bussola', 'DVL', 'camera', 'IMU'], 2022, 'Lua', 42, 250)
brhue = Auv(6, ['profundidade', 'sonoro', 'camera', 'IMU'], 2020, 'BrHue', 35, 225)
lista_auv = [brhue, lua]

# while para mostrar e escolher as opções de exibição 
while True:
    clearscreen()
    print("Escolha uma opção:")
    print("1 - Exibir todos")
    print("2 - Exibir Lua")
    print("3 - Exibir BrHue")
    print("4 - Exibir ranking por ano (decrescente)")
    print("5 - Exibir ranking por tempo de teste (crescente)")
    print("0 - Sair\n")
    
    choice = input("Digite o número da opção desejada: ")
    
    if choice == "1":
        clearscreen()
        print("\nExibindo todos os AUVs:\n")
        Auv.exibir_todos(lista_auv)
        print("\n")
    elif choice == "2":
        clearscreen()
        print("\nExibindo informações do AUV Lua:\n")
        Auv.exibir_auv(lua)
        print("\n")
    elif choice == "3":
        clearscreen()
        print("\nExibindo informações do AUV BrHue:\n")
        Auv.exibir_auv(brhue)
        print("\n")
    elif choice == "4":
        clearscreen()
        print("\nExibindo ranking dos AUVs:\n")
        Auv.rank_ano(lista_auv)
        print("\n")
    elif choice == "5":
        clearscreen()
        print("\nExibindo ranking dos AUVs:\n")
        Auv.rank_teste(lista_auv)
        print("\n")
    elif choice == "0":
        break
    else:
        clearscreen()
        print("\nOpção inválida. Tente novamente.\n")
    
    input("Pressione Enter para continuar...")

print("Saindo...")
