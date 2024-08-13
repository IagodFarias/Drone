#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
public:
    KalmanFilter(int state_dim, int meas_dim) {
        x = VectorXd::Zero(state_dim);   // Estado estimado
        P = MatrixXd::Identity(state_dim, state_dim);  // Covariância do estado
        A = MatrixXd::Identity(state_dim, state_dim);  // Modelo de transição do estado
        H = MatrixXd::Zero(meas_dim, state_dim);       // Modelo de observação
        Q = MatrixXd::Identity(state_dim, state_dim);  // Covariância do processo
        R = MatrixXd::Identity(meas_dim, meas_dim);    // Covariância da medição
    }

    void predict(const VectorXd& u = VectorXd()) {
        x = A * x + B * u;   // Previsão do estado
        P = A * P * A.transpose() + Q;   // Previsão da covariância
    }

    void update(const VectorXd& z) {
        VectorXd y = z - H * x;   // Inovação
        MatrixXd S = H * P * H.transpose() + R;   // Covariância da inovação
        MatrixXd K = P * H.transpose() * S.inverse();   // Ganho de Kalman

        x = x + K * y;   // Atualização do estado
        P = (MatrixXd::Identity(x.size(), x.size()) - K * H) * P;   // Atualização da covariância
    }

    void setA(const MatrixXd& A) { this->A = A; }
    void setB(const MatrixXd& B) { this->B = B; }
    void setH(const MatrixXd& H) { this->H = H; }
    void setQ(const MatrixXd& Q) { this->Q = Q; }
    void setR(const MatrixXd& R) { this->R = R; }
    VectorXd getState() const { return x; }

private:
    VectorXd x;  // Estado
    MatrixXd P;  // Covariância do estado
    MatrixXd A;  // Modelo de transição do estado
    MatrixXd B;  // Modelo de controle
    MatrixXd H;  // Modelo de observação
    MatrixXd Q;  // Covariância do processo
    MatrixXd R;  // Covariância da medição
};

int main() {
    KalmanFilter kf(2, 1);  // Exemplo: sistema com 2 estados e 1 medição

    // Inicializar matrizes A, B, H, Q, R conforme necessário
    // Aqui está um exemplo simples:
    MatrixXd A(2, 2);
    A << 1, 1, 0, 1;
    kf.setA(A);

    MatrixXd H(1, 2);
    H << 1, 0;
    kf.setH(H);

    MatrixXd Q = 0.1 * MatrixXd::Identity(2, 2);
    kf.setQ(Q);

    MatrixXd R(1, 1);
    R << 1;
    kf.setR(R);

    VectorXd z(1);
    z << 10;  // Exemplo de medição

    // Prever e atualizar o filtro
    kf.predict();
    kf.update(z);

    // Obter o estado estimado
    VectorXd x = kf.getState();
    std::cout << "Estado estimado:\n" << x << std::endl;

    return 0;
}
