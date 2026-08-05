# Diretrizes de Escrita e Formatação da Dissertação (LaTeX)

Este documento reúne as regras obrigatórias de redação e estruturação para o desenvolvimento da tese. Deves consultar este ficheiro antes e durante a escrita de cada secção para garantir a máxima qualidade e conformidade com as regras estabelecidas.

---

## ✍️ 1. Estilo de Redação e Sintaxe

- [ ] **Frases Curtas e Focadas**: Evita frases muito longas ou parágrafos de frase única. Utiliza **apenas uma ideia principal por frase**.
- [ ] **Consistência de Terminologia**: É obrigatório usar sempre a mesma denominação para o mesmo assunto. 
  - *Exemplo*: Se escolheres o termo "área", usa sempre "área" em todo o documento. Não alternes com sinónimos como "zona" ou "localização".
- [ ] **Consistência de Notação Matemática**: Utiliza rigorosamente a mesma notação e simbologia matemática em todo o documento (ex: negrito para matrizes/vetores $\mathbf{x}$, itálico para escalares $x$, e os mesmos índices para identificar os veículos: $_v$ para USV e $_a$ para UAV).
- [ ] **Precedência de Conceitos e Siglas**: Nunca menciones conceitos, termos técnicos ou siglas que não tenham sido formalmente apresentados ou definidos anteriormente no texto. Introduz sempre as siglas na primeira ocorrência (ex: *Nonlinear Model Predictive Control* (NMPC)).
- [ ] **Escrita Objetiva**: Mantém a escrita técnica e direta. Não tenhas receio de repetir palavras-chave se isso mantiver a precisão e clareza do texto.
- [ ] **Motivação e Clareza de Decisões**: Escrever num tom explicativo para o leitor acompanhar facilmente

---

## 📊 2. Regras para Figuras e Tabelas

- [ ] **Prioridade à Visualização**: Tudo o que puder ter uma representação visual (esquemas, blocos diagramáticos, fluxogramas, gráficos ou tabelas) deve ter. Evita blocos densos de texto descritivo se a informação puder ser transmitida visualmente de forma mais clara.
- [ ] **Identificação Completa**: Todas as figuras e tabelas têm de ter um **número único** e uma **legenda descritiva** (caption).
- [ ] **Índice de Elementos**: Devem estar listadas nos respetivos índices automáticos no início da tese (`\listoffigures` e `\listoftables`).
- [ ] **Referenciação Obrigatória**: Cada figura ou tabela inserida tem de ser obrigatoriamente referenciada no corpo de texto pelo menos uma vez (ex: `Fig.~\ref{fig:coordinate_frames}`) e devidamente explicada.
- [ ] **Referência pelo Número**: Nunca te refiras a uma imagem como "a figura seguinte", "a figura abaixo" ou "a tabela acima". Utiliza sempre a referência numérica direta (ex: `Figure 3.1` ou `Table 3.2`).
- [ ] **Posicionamento Relativo**: A figura/tabela deve ser colocada **apenas após** ser referenciada no texto pela primeira vez (nunca antes).
- [ ] **Sem Elementos Flutuantes no Início**: Uma figura ou tabela **nunca** deve ser o primeiro elemento no início de um capítulo ou secção. Deve existir sempre texto introdutório antes de qualquer imagem ou tabela.

---

## 🏛️ 3. Estrutura Padrão dos Capítulos

Cada capítulo da dissertação deve começar obrigatoriamente com um **parágrafo introdutório** contendo:

1. **Objetivo do Capítulo**: Uma única frase inicial que defina claramente o propósito daquele capítulo.
2. **Metodologia de Desenvolvimento**: Uma explicação concisa de como o trabalho desse capítulo foi desenvolvido.
3. **Fio Condutor / Estrutura**: Um descritivo das subsecções que compõem o capítulo, guiando o leitor sobre a sequência lógica dos conteúdos apresentados.

---

## 🛠️ Exemplo de Referenciação em LaTeX

```latex
% Exemplo de texto introdutório antes da figura:
As transformações cinemáticas entre o referencial inercial e os referenciais locais de cada veículo são apresentadas na Figure~\ref{fig:coordinate_frames}.

\begin{figure}[H]
    \centering
    % Código do TikZ ou \includegraphics
    \caption{Representação esquemática dos referenciais e acoplamento do cabo.}
    \label{fig:coordinate_frames}
\end{figure}

% Explicar a figura logo a seguir:
Como se observa na Figure~\ref{fig:coordinate_frames}, a âncora do cabo...
```
