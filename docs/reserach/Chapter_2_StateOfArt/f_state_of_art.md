## 1) Research Strategy

<!-- GitHub render -->
![Query Logic Diagram](/docs/reserach/illustrations/query_logic.drawio.png)

### Categories
- $C_1  =$ Classical / Observers
- $C_2  =$ Gaussian Process (GP)
- $C_3  =$ Deep Sequence Models (MLP / GRU / TCN / Transformer / LSTM)
- $C_4  =$ Physics-Informed / Differentiable  
- $C_5  =$ Surveys 
- $C_T  =$ Goal & Domain Terms
  - $Cmt =$ Estimation & Modeling Terms
  - $Cct =$ Robotics Context Terms

### Query Logic (Generalized Set Intersection)

### Combined Representation

$$
C = \{ C_1, \dots, C_4 \}
$$

$$
Q = \bigcup_{i=1}^{5} Q_i
$$

$$
Q_i = \left( \bigvee_{c \in C_i} c \right) 
\;\; \land \;\; 
\left( \bigvee_{e \in Cmt} e \right) 
\;\; \land \;\; 
\left( \bigvee_{r \in Cct} r \right),
\quad i = 1, 2, \dots, 5
$$


# Research Results Summary

## Results in Numbers / Research Trend

[See detailed results](research_trend.md)

---

| Query                                 | Relev. SoA | Rigid-body | Payload | Both  |
| ---------------------------------     | ---------- | ---------- | ------- | ----- |
| **$Q_1  =$ Classical / Observers**    | 17         | 7          | 7       | 3     |
| **$Q_2  =$ Gaussian Process (GP)**    | 4          | 4          | 0       | 0     |
| **$Q_3  =$ Deep Sequence Models**     | 8          | 4          | 4       | 0     |
| **$Q_4  =$ Physics-Informed / Diff.** | 5          | 5          | 0       | 0     |
| **$Q_5  =$ Surveys**                  | 2          | –          | –       | –     |
| **Total**                             | **36**     | **20**     | **11**  | **3** |

