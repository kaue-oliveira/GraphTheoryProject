-- Trabalho Prático
-- Universidade Federal de Lavras
-- Curso: Ciência da Computação
-- Disciplina: Algoritmos em Grafos
-- Grupo K
-- Membro 1: Kauê de Oliveira Silva    nº de matrícula: 202310532  Turma: 10 A
-- Membro 2: Thiago Ferreira Azevedo   nº de matrícula: 202311097  Turma: 10 A    

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <string>
#include <algorithm>
#include <limits.h>
#include <sstream>
#include <numeric>
#include <set>
#include <chrono>
using namespace std;


class Grafo {
private:
    int numvertices;
    bool direcionado;
    vector<int> vertices;
    vector<int> arestas;
    vector<vector<pair<int, int>>> listaAdjacencia; 
    vector<pair<int, int>> arestasTotais; 
    vector<vector<pair<int, int>>> listaAdjacenciaTransposta; // Adicionando a lista de adjacência transposta

    void dfsChoro(int v, vector<bool>& visitado, stack<int>& pilha);
    void dfsTransposta(int v, vector<bool>& visitado, stack<int>& componentePilha);
public:
    Grafo(int vertices, bool direcionado) : numvertices(vertices), direcionado(direcionado) {
        listaAdjacencia.resize(vertices); 
        if (direcionado) {
            listaAdjacenciaTransposta.resize(vertices); // Inicializa a lista de adjacência transposta
        }
    }

    

    void inserearesta(size_t indice, int Nosaida, int Noentrada, int peso) {
      int linha = Nosaida;
      int coluna = Noentrada;
      listaAdjacencia[linha].push_back({coluna, peso});
      if (!direcionado) {
          listaAdjacencia[coluna].push_back({linha, peso});
      }
      if (indice <= arestasTotais.size()) {
          arestasTotais.insert(arestasTotais.begin() + indice, std::make_pair(linha, coluna));
      }
}


    void inserearestaDepre(int Nosaida, int Noentrada, int peso) {
        int linha = Nosaida;
        int coluna = Noentrada;
        
        listaAdjacencia[linha].push_back({coluna, peso});
        
        if (!direcionado) {
            listaAdjacencia[coluna].push_back({linha, peso});
        }
    }


bool ehConexo();
bool ehBipartido();
bool ehEuleriano();
bool possuiCiclos();
bool possuiCiclosUtil(int v, vector<bool>& visitado, vector<bool>& recStack);
int contarComponentesConexas();
int contarComponentesFortementeConexas();
void encontrarVerticesArticulacao();
void encontrarArestasPonte();
void buscaemprofundidade();
void buscaemlargura();
void prim();
void exibirOrdemTopologica();
void exibirCaminhoMinimoDijkstra();
int fluxoMaximo();
void DFS(int v, vector<bool>& visitado);
void articulationUtil(int u, vector<bool>& visitado, vector<int>& disc, vector<int>& low, vector<int>& parent, vector<bool>& articulation);
void bridgeUtil(int u, vector<bool>& visitado, vector<int>& disc, vector<int>& low, vector<int>& parent);
void buscaemprofundidadeUtil(int v, vector<bool>& visitado);
void topologicalSortUtil(int v, vector<bool>& visitado, stack<int>& Stack);
void fechoTransitivo();
bool possuiCiclosUtilNaoDirecionado(int v, vector<bool>& visitado, int parent);
void dfs(int v, vector<bool>& visitado, vector<int>& idArestaPercorrida);
void dfsFecho(int v, vector<bool>& visitado, vector<int>& verticesAlcancados);
};

bool Grafo::ehConexo() {
    if (!direcionado) {
        if (numvertices == 0) return true;

        vector<bool> visitados(numvertices, false);
        queue<int> fila;
        fila.push(0);
        visitados[0] = true;

        while (!fila.empty()) {
            int verticeAtual = fila.front();
            fila.pop();

            for (const auto& adj : listaAdjacencia[verticeAtual]) {
                int adjVertice = adj.first;
                if (!visitados[adjVertice]) {
                    fila.push(adjVertice);
                    visitados[adjVertice] = true;
                }
            }
        }

        return all_of(visitados.begin(), visitados.end(), [](bool v) { return v; });
    } else {
        vector<vector<int>> grafoNaoDirecionado(numvertices);

        for (int i = 0; i < numvertices; ++i) {
            for (const auto& adj : listaAdjacencia[i]) {
                int v = adj.first;
                grafoNaoDirecionado[i].push_back(v);
                grafoNaoDirecionado[v].push_back(i);
            }
        }

        vector<bool> visitados(numvertices, false);
        queue<int> fila;
        fila.push(0);
        visitados[0] = true;

        while (!fila.empty()) {
            int verticeAtual = fila.front();
            fila.pop();

            for (int adjVertice : grafoNaoDirecionado[verticeAtual]) {
                if (!visitados[adjVertice]) {
                    fila.push(adjVertice);
                    visitados[adjVertice] = true;
                }
            }
        }

        return all_of(visitados.begin(), visitados.end(), [](bool v) { return v; });
    }
}




bool Grafo::ehBipartido() {
    vector<int> cor(numvertices, -1);
    queue<int> q;
    
    for (int i = 0; i < numvertices; ++i) {
        if (cor[i] == -1) {
            cor[i] = 0;
            q.push(i);
            
            while (!q.empty()) {
                int v = q.front();
                q.pop();
                
                for (const auto& adj : listaAdjacencia[v]) {
                    int u = adj.first;
                    if (cor[u] == -1) {
                        cor[u] = 1 - cor[v];
                        q.push(u);
                    } else if (cor[u] == cor[v]) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool Grafo::possuiCiclosUtil(int v, vector<bool>& visitado, vector<bool>& recStack) {
    visitado[v] = true;
    recStack[v] = true;
    
    for (const auto& adj : listaAdjacencia[v]) {
        int u = adj.first;
        if (!visitado[u] && possuiCiclosUtil(u, visitado, recStack)) {
            return true;
        } else if (recStack[u]) {
            return true;
        }
    }
    
    recStack[v] = false;
    return false;
}

bool Grafo::possuiCiclosUtilNaoDirecionado(int v, vector<bool>& visitado, int parent) {
    visitado[v] = true;

    for (const auto& adj : listaAdjacencia[v]) {
        int u = adj.first;
        if (!visitado[u]) {
            if (possuiCiclosUtilNaoDirecionado(u, visitado, v)) {
                return true;
            }
        } else if (u != parent) {
            return true;
        }
    }

    return false;
}

bool Grafo::possuiCiclos() {
    if (direcionado) {
        vector<bool> visitado(numvertices, false);
        vector<bool> recStack(numvertices, false);
        
        for (int i = 0; i < numvertices; ++i) {
            if (!visitado[i] && possuiCiclosUtil(i, visitado, recStack)) {
                return true;
            }
        }
        return false;
    } else {
        vector<bool> visitado(numvertices, false);
        
        for (int i = 0; i < numvertices; ++i) {
            if (!visitado[i]) {
                if (possuiCiclosUtilNaoDirecionado(i, visitado, -1)) {
                    return true;
                }
            }
        }
        return false;
    }
}

void Grafo::DFS(int v, vector<bool>& visitado) {
    visitado[v] = true;
    for (const auto& adj : listaAdjacencia[v]) {
        int u = adj.first;
        if (!visitado[u]) {
            DFS(u, visitado);
        }
    }
}

int Grafo::contarComponentesConexas() {
  if(!direcionado){
    vector<bool> visitado(numvertices, false);
    int cont = 0;
    
    for (int i = 0; i < numvertices; ++i) {
        if (!visitado[i]) {
            DFS(i, visitado);
            ++cont;
        }
    }
    
    return cont;
  } else {
    return -1;
  }
}


bool Grafo::ehEuleriano() {
    if (direcionado) {
        for (int i = 0; i < numvertices; ++i) {
            int grauEntrada = 0, grauSaida = 0;
            for (int j = 0; j < numvertices; ++j) {
                for (const auto& adj : listaAdjacencia[j]) {
                    if (adj.first == i) grauEntrada++;
                }
                if (j == i) grauSaida = listaAdjacencia[j].size();
            }
            if (grauEntrada != grauSaida) return false;
        }

        return contarComponentesFortementeConexas() == 1;
    } else {
        for (int i = 0; i < numvertices; ++i) {
            if (listaAdjacencia[i].size() % 2 != 0) return false;
        }

        return ehConexo();
    }
}

void Grafo::dfsChoro(int v, vector<bool>& visitado, stack<int>& pilha) {
    visitado[v] = true;
    for (const auto& adj : listaAdjacencia[v]) {
        int u = adj.first;
        if (!visitado[u]) {
            dfsChoro(u, visitado, pilha);
        }
    }
    pilha.push(v);
}

void Grafo::dfsTransposta(int v, vector<bool>& visitado, stack<int>& componentePilha) {
    visitado[v] = true;
    componentePilha.push(v);
    
    // Ordena os adjacentes do grafo transposto para garantir a ordem desejada
    vector<pair<int, int>> adjacentes = listaAdjacenciaTransposta[v];
    sort(adjacentes.begin(), adjacentes.end());
    
    for (const auto& adj : adjacentes) {
        int u = adj.first;
        if (!visitado[u]) {
            dfsTransposta(u, visitado, componentePilha);
        }
    }
}

int Grafo::contarComponentesFortementeConexas() {
    if (!direcionado) {
        return -1; // Não aplicável para grafos não direcionados
    }

    stack<int> pilha;
    vector<bool> visitado(numvertices, false);

    // Passo 1: Preenchimento da pilha com a ordem de término da DFS
    for (int i = 0; i < numvertices; ++i) {
        if (!visitado[i]) {
            dfsChoro(i, visitado, pilha);
        }
    }

    // Passo 2: Transpor o grafo
    listaAdjacenciaTransposta = vector<vector<pair<int, int>>>(numvertices);
    for (int v = 0; v < numvertices; ++v) {
        for (const auto& adj : listaAdjacencia[v]) {
            listaAdjacenciaTransposta[adj.first].emplace_back(v, 0);
        }
    }

    // Passo 3: DFS no grafo transposto na ordem inversa da pilha
    fill(visitado.begin(), visitado.end(), false);
    int cont = 0;
    
    while (!pilha.empty()) {
        int v = pilha.top();
        pilha.pop();
        if (!visitado[v]) {
            stack<int> componentePilha;
            dfsTransposta(v, visitado, componentePilha);
            
            // Contar apenas componentes com mais de um vértice
            if (componentePilha.size() >= 1) {
                cont++;
            }
        }
    }

    return cont;
}


void Grafo::articulationUtil(int u, vector<bool>& visitado, vector<int>& disc, vector<int>& low, vector<int>& parent, vector<bool>& articulation) {
    static int time = 0;
    int filhos = 0;
    visitado[u] = true;
    disc[u] = low[u] = ++time;

    for (const auto& adj : listaAdjacencia[u]) {
        int v = adj.first;
        
        if (!visitado[v]) {
            ++filhos;
            parent[v] = u;
            articulationUtil(v, visitado, disc, low, parent, articulation);
            low[u] = min(low[u], low[v]);
            
            if (parent[u] == -1 && filhos > 1) {
                articulation[u] = true;
            }
            if (parent[u] != -1 && low[v] >= disc[u]) {
                articulation[u] = true;
            }
        } else if (v != parent[u]) {
            low[u] = min(low[u], disc[v]);
        }
    }
}

void Grafo::encontrarVerticesArticulacao() {
  if (!direcionado){
    vector<bool> visitado(numvertices, false);
    vector<int> disc(numvertices, -1);
    vector<int> low(numvertices, -1);
    vector<int> parent(numvertices, -1);
    vector<bool> articulation(numvertices, false);
    bool teste = false;
    for (int i = 0; i < numvertices; ++i) {
        if (!visitado[i]) {
            articulationUtil(i, visitado, disc, low, parent, articulation);
        }
    }
    for (int i = 0; i < numvertices; ++i) {
        if (articulation[i] == 1) {
          teste = true;
            cout << i << " ";
        }
    }
    if(!teste){
      cout << 0;
    }
    cout << endl;
  } else {
    cout << -1 << endl;
  }
}
void Grafo::dfs(int v, vector<bool>& visitado, vector<int>& idArestaPercorrida) {
    visitado[v] = true;
    
    vector<pair<int, int>> adjacentes = listaAdjacencia[v];
    sort(adjacentes.begin(), adjacentes.end());
    
    for (const auto& adj : adjacentes) {
        int u = adj.first;
        if (!visitado[u]) {
            for (size_t i = 0; i < arestasTotais.size(); ++i) {
                    if ((arestasTotais[i].first == v && arestasTotais[i].second == u) ||
                    (!direcionado && arestasTotais[i].first == u && arestasTotais[i].second == v)) {
                    idArestaPercorrida.push_back(i);
                    break;
                }
            }
            dfs(u, visitado, idArestaPercorrida);
        }
    }
}

// Método buscaemprofundidade atualizado
void Grafo::buscaemprofundidade() {
    vector<bool> visitado(numvertices, false);
    vector<int> idArestaPercorrida;
    
    dfs(0, visitado, idArestaPercorrida); 
    
    for (int id : idArestaPercorrida) {
        cout << id << " ";
    }
    cout << endl;
}
void Grafo::encontrarArestasPonte() {
    if (!direcionado) {
        vector<int> disc(numvertices, -1);
        vector<int> low(numvertices, -1);
        vector<int> pai(numvertices, -1);
        vector<int> pontes;
        int tempo = 0;

        auto DFSParaPontes = [&](int u, int p, auto& self) -> void {
            disc[u] = low[u] = ++tempo;

            for (const auto& adj : listaAdjacencia[u]) {  // Correção aqui
                int v = adj.first;

                if (v == p) continue; 

                if (disc[v] == -1) { 
                    pai[v] = u;
                    self(v, u, self);

                    low[u] = min(low[u], low[v]);

                    if (low[v] > disc[u]) {
                  for (size_t i = 0; i < arestasTotais.size(); ++i) {
                            if ((arestasTotais[i].first == u && arestasTotais[i].second == v) ||
                                (arestasTotais[i].first == v && arestasTotais[i].second == u)) {
                                pontes.push_back(i);
                                break;
                            }
                        }
                    }
                } else if (v != pai[u]) {
                    low[u] = min(low[u], disc[v]);
                }
            }
        };

        for (int i = 0; i < numvertices; ++i) {
            if (disc[i] == -1) {
                DFSParaPontes(i, -1, DFSParaPontes);
            }
        }

        sort(pontes.begin(), pontes.end()); 

        if (!pontes.empty()) {
            for (int id : pontes) {
                cout << id << " ";
            }
        } else {
            cout << "-1";
        }
        cout << endl;
    } else {
        cout << "-1" << endl;
    }
}

void Grafo::buscaemlargura() {
    vector<bool> visitado(numvertices, false);
    vector<pair<int, int>> arestasPercorridas; 
    vector<int> idArestaPercorrida; 
    queue<int> q;
    
    q.push(0);
    visitado[0] = true;
    
    while (!q.empty()) {
        int v = q.front();
        q.pop();
        
        for (const auto& adj : listaAdjacencia[v]) {
            int u = adj.first;
            if (!visitado[u]) {
                visitado[u] = true;
                q.push(u);
                
                arestasPercorridas.push_back({v, u});
                
                for (size_t i = 0; i < arestasTotais.size(); ++i) {
                    if ((arestasTotais[i].first == v && arestasTotais[i].second == u) ||
                        (!direcionado && arestasTotais[i].first == u && arestasTotais[i].second == v)) {
                        idArestaPercorrida.push_back(i);
                        break;
                    }
                }
            }
        }
    }

    for (int id : idArestaPercorrida) {
        cout << id << " ";
    }
    cout << endl;
}

void Grafo::prim() {
  if (direcionado) {
      cout << -1 << endl;
      return;
  }
    if (ehConexo()){
    vector<int> chave(numvertices, INT_MAX);
    vector<int> pai(numvertices, -1);
    vector<bool> noMST(numvertices, false);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    
    chave[0] = 0;
    pq.push({0, 0});
    
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        
        noMST[u] = true;
        
        for (const auto& adj : listaAdjacencia[u]) {
            int v = adj.first;
            int peso = adj.second;
            
            if (!noMST[v] && peso < chave[v]) {
                chave[v] = peso;
                pq.push({peso, v});
                pai[v] = u;
            }
        }
    }
    
    int soma = 0;
    for (int i = 1; i < numvertices; ++i) {
        soma += chave[i];
    }
    cout << soma << endl;
  } else {
    cout << -1 << endl;
  }
}

void Grafo::topologicalSortUtil(int v, vector<bool>& visitado, stack<int>& Stack) {
    visitado[v] = true;
    
    for (const auto& adj : listaAdjacencia[v]) {
        int u = adj.first;
        if (!visitado[u]) {
            topologicalSortUtil(u, visitado, Stack);
        }
    }
    
    Stack.push(v);
}

void Grafo::exibirOrdemTopologica() {
  if (direcionado){
    if (!possuiCiclos()){
    stack<int> Stack;
    vector<bool> visitado(numvertices, false);
    
    for (int i = 0; i < numvertices; ++i) {
        if (!visitado[i]) {
            topologicalSortUtil(i, visitado, Stack);
        }
    }
    
    while (!Stack.empty()) {
        cout << Stack.top() << " ";
        Stack.pop();
    }
    cout << endl;
  } else {
    cout << -1 << endl;
  }
  } else {
    cout << -1 << endl;
  }
}

void Grafo::exibirCaminhoMinimoDijkstra() {
    vector<int> dist(numvertices, INT_MAX);
    vector<int> pai(numvertices, -1);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    
    dist[0] = 0;
    pq.push({0, 0});
    
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        
        for (const auto& adj : listaAdjacencia[u]) {
            int v = adj.first;
            int peso = adj.second;
            
            if (dist[u] + peso < dist[v]) {
                dist[v] = dist[u] + peso;
                pq.push({dist[v], v});
                pai[v] = u;
            }
        }
    }
    
    if (dist[numvertices - 1] == INT_MAX) {
        cout << "-1" << endl;
    } else {
        cout << dist[numvertices - 1] << endl;
    }
}

int Grafo::fluxoMaximo() {
    if (direcionado) {
        // Cria a matriz de capacidades e o grafo residual
        vector<vector<int>> capacidade(numvertices, vector<int>(numvertices, 0));
        vector<vector<int>> grafoResidual(numvertices, vector<int>(numvertices, 0));

        // Inicializa a capacidade do grafo residual com as capacidades originais
        for (int u = 0; u < numvertices; ++u) {
            for (const auto& [v, peso] : listaAdjacencia[u]) {
                capacidade[u][v] = peso;
                grafoResidual[u][v] = peso;
            }
        }

        // Função auxiliar para realizar a busca em largura (BFS)
        auto bfs = [&](int fonte, int sink, vector<int>& pai) -> bool {
            fill(pai.begin(), pai.end(), -1);
            pai[fonte] = fonte;
            queue<pair<int, int>> fila;
            fila.push({fonte, INT_MAX});

            while (!fila.empty()) {
                int v = fila.front().first;
                int fluxo = fila.front().second;
                fila.pop();

                for (const auto& adj : listaAdjacencia[v]) {
                    int u = adj.first;
                    if (pai[u] == -1 && grafoResidual[v][u] > 0) {
                        pai[u] = v;
                        int novoFluxo = min(fluxo, grafoResidual[v][u]);
                        if (u == sink) {
                            return true;
                        }
                        fila.push({u, novoFluxo});
                    }
                }
            }
            return false;
        };

        int fonte = 0;
        int sink = numvertices - 1;
        int fluxoMaximo = 0;
        vector<int> pai(numvertices);

        // Implementa o algoritmo de Ford-Fulkerson
        while (bfs(fonte, sink, pai)) {
            int fluxoCaminho = INT_MAX;
            for (int v = sink; v != fonte; v = pai[v]) {
                int u = pai[v];
                fluxoCaminho = min(fluxoCaminho, grafoResidual[u][v]);
            }

            for (int v = sink; v != fonte; v = pai[v]) {
                int u = pai[v];
                grafoResidual[u][v] -= fluxoCaminho;
                grafoResidual[v][u] += fluxoCaminho;
            }

            fluxoMaximo += fluxoCaminho;
        }

        return fluxoMaximo;
    } else {
        return -1; // Não aplicável para grafos não direcionados
    }
}
// Função auxiliar DFS
void Grafo::dfsFecho(int v, vector<bool>& visitado, vector<int>& verticesAlcancados) {
    visitado[v] = true;
    verticesAlcancados.push_back(v);

    vector<pair<int, int>> adjacentes = listaAdjacencia[v];
    sort(adjacentes.begin(), adjacentes.end());

    for (const auto& adj : adjacentes) {
        int u = adj.first;
        if (!visitado[u]) {
            dfsFecho(u, visitado, verticesAlcancados);
        }
    }
}
// Método fechoTransitivo atualizado
void Grafo::fechoTransitivo() {
    if (!direcionado) {
        cout << -1 << endl;
        return;
    }

    vector<bool> visitado(numvertices, false);
    vector<int> verticesAlcancados;

    // Inicia a busca em profundidade a partir do vértice 0
    dfsFecho(0, visitado, verticesAlcancados);

    // Exibe os vértices alcançados
    for (int vertice : verticesAlcancados) {
      if (vertice != 0)
        cout << vertice << " ";
    }
    cout << endl;
}

int main() {
    vector<int> comandos;
    string linha;
    
    getline(cin, linha);
    istringstream iss(linha);
    
    int num;
    while (iss >> num) {
        comandos.push_back(num);
    }

  
  int vertices, arestas;
  cin >> vertices >> arestas;
  string ehdirecionado;
  cin >> ehdirecionado;
  
  Grafo grafo(vertices, ehdirecionado == "direcionado");
for (int i = 0; i < arestas; i++) {
    int a, u, v, peso;
    cin >> a >> u >> v >> peso;
    grafo.inserearesta(a, u, v, peso);
}
    for (int comando : comandos) {
        if (comando == 0) {
            cout << grafo.ehConexo() << endl;
        } else if (comando == 1) {
            cout << grafo.ehBipartido() << endl;
        } else if (comando == 2) {
            cout << grafo.ehEuleriano() << endl;
        } else if (comando == 3) {
            cout << grafo.possuiCiclos() << endl;
        } else if (comando == 4) {
           cout << grafo.contarComponentesConexas() << endl;
        } else if (comando == 5) {
            cout << grafo.contarComponentesFortementeConexas() << endl;
        } else if (comando == 6) {
            grafo.encontrarVerticesArticulacao();
        } else if (comando == 7) {
            grafo.encontrarArestasPonte();
        } else if (comando == 8) {
            grafo.buscaemprofundidade();
        } else if (comando == 9) {
            grafo.buscaemlargura();
        } else if (comando == 10) {
            grafo.prim();
        } else if (comando == 11) {
            grafo.exibirOrdemTopologica();
        } else if (comando == 12) {
            grafo.exibirCaminhoMinimoDijkstra();
        } else if (comando == 13) {
            cout << grafo.fluxoMaximo() << endl;
        } else if (comando == 14) {
            grafo.fechoTransitivo();
        } else {
            cout << "Comando inválido: " << comando << endl;
        }
    }

    return 0;
}

