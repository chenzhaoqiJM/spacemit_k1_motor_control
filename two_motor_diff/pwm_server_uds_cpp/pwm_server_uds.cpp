#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <thread>
#include <chrono>
#include <cstring>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ================== PWM 配置 ==================
struct PWMConfig {
    std::string chip;
    int index;
    std::string base;
};

static const std::map<int, PWMConfig> PWM_TEMPLATE = {
    {1, {"/sys/class/pwm/pwmchip0", 0, ""}},
    {2, {"/sys/class/pwm/pwmchip1", 0, ""}},
};

static constexpr int PERIOD_NS = 1'000'000;  // 1 kHz
static const char* SOCK_PATH = "/run/pwm_control_uds.sock";

// ================== 文件工具 ==================
void write_file(const std::string& path, const std::string& value) {
    std::ofstream ofs(path);
    if (!ofs) {
        throw std::runtime_error("open failed: " + path);
    }
    ofs << value;
}

std::string read_file(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) {
        throw std::runtime_error("open failed: " + path);
    }
    std::string val;
    ifs >> val;
    return val;
}

std::string pwm_base(const std::string& chip, int index) {
    return chip + "/pwm" + std::to_string(index);
}

void ensure_pwm_exported(const std::string& chip, int index) {
    std::string base = pwm_base(chip, index);
    if (access(base.c_str(), F_OK) == 0) {
        return;
    }

    write_file(chip + "/export", std::to_string(index));

    for (int i = 0; i < 20; ++i) {
        if (access(base.c_str(), F_OK) == 0) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    throw std::runtime_error("PWM export timeout: " + base);
}

// ================== 主程序 ==================
int main() {
    if (geteuid() != 0) {
        std::cerr << "Must be run as root\n";
        return 1;
    }

    // ---------- PWM 初始化 ----------
    std::map<int, PWMConfig> pwms = PWM_TEMPLATE;

    try {
        for (auto& [id, cfg] : pwms) {
            ensure_pwm_exported(cfg.chip, cfg.index);
            cfg.base = pwm_base(cfg.chip, cfg.index);

            std::string enable = cfg.base + "/enable";

            if (read_file(enable) != "0") {
                write_file(enable, "0");
            }

            write_file(cfg.base + "/period", std::to_string(PERIOD_NS));
            write_file(cfg.base + "/duty_cycle", "0");
            write_file(enable, "1");
        }
    } catch (const std::exception& e) {
        std::cerr << "PWM init failed: " << e.what() << "\n";
        return 1;
    }

    std::cout << "PWM initialized: pwm_motor_1, pwm_motor_2\n";

    // ---------- UNIX Domain Socket ----------
    unlink(SOCK_PATH);

    int srv_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv_fd < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(srv_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(srv_fd);
        return 1;
    }

    chmod(SOCK_PATH, 0666);

    if (listen(srv_fd, 1) < 0) {
        perror("listen");
        close(srv_fd);
        return 1;
    }

    std::cout << "PWM UDS server listening: " << SOCK_PATH << "\n";

    // ---------- 主循环 ----------
    while (true) {
        int conn_fd = accept(srv_fd, nullptr, nullptr);
        if (conn_fd < 0) {
            perror("accept");
            continue;
        }

        std::cout << "client connected\n";

        while (true) {
            char buf[256];
            ssize_t n = recv(conn_fd, buf, sizeof(buf), 0);
            if (n <= 0) {
                break;
            }

            json resp;
            try {
                json msg = json::parse(std::string(buf, n));

                double duty1 = msg.value("duty_motor1", 0.0);
                double duty2 = msg.value("duty_motor2", 0.0);

                duty1 = std::clamp(duty1, 0.0, 1.0);
                duty2 = std::clamp(duty2, 0.0, 1.0);

                write_file(
                    pwms[1].base + "/duty_cycle",
                    std::to_string(static_cast<int>(PERIOD_NS * duty1))
                );
                write_file(
                    pwms[2].base + "/duty_cycle",
                    std::to_string(static_cast<int>(PERIOD_NS * duty2))
                );

                resp["ok"] = true;
            } catch (const std::exception& e) {
                resp["ok"] = false;
                resp["error"] = e.what();
            }

            try {
                std::string out = resp.dump();
                send(conn_fd, out.c_str(), out.size(), 0);
            } catch (...) {
                break;
            }
        }

        close(conn_fd);
        std::cout << "client disconnected\n";
    }

    close(srv_fd);
    return 0;
}
