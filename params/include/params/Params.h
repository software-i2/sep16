// Copyright by BeeX [2026]

#ifndef PARAMS_PARAMS_H
#define PARAMS_PARAMS_H

#include <ros/param.h>

#include <set>
#include <string>
#include <vector>

namespace params {

// Strict reads from one namespace of the parameter server: nothing is defaulted, every problem is recorded.
class Params {
public:
    explicit Params(const std::string &ns) : ns_(ns) {}

    double number(const std::string &key) {
        XmlRpc::XmlRpcValue value;
        double              out = 0.0;
        if (fetch(key, value) && !toNumber(value, out)) {
            wrongType(key, "a number");
        }
        return out;
    }

    int whole(const std::string &key) {
        XmlRpc::XmlRpcValue value;
        if (!fetch(key, value)) {
            return 0;
        }
        if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
            wrongType(key, "a whole number");
            return 0;
        }
        return static_cast<int>(value);
    }

    bool flag(const std::string &key) {
        XmlRpc::XmlRpcValue value;
        if (!fetch(key, value)) {
            return false;
        }
        if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
            wrongType(key, "true or false");
            return false;
        }
        return static_cast<bool>(value);
    }

    std::string text(const std::string &key) {
        XmlRpc::XmlRpcValue value;
        if (!fetch(key, value)) {
            return std::string();
        }
        if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            wrongType(key, "text");
            return std::string();
        }
        return static_cast<std::string>(value);
    }

    std::vector<double> numbers(const std::string &key, size_t count) {
        XmlRpc::XmlRpcValue value;
        std::vector<double> out(count, 0.0);
        if (!fetch(key, value)) {
            return out;
        }
        if (!toNumbers(value, count, out)) {
            wrongType(key, "a list of " + std::to_string(count) + " numbers");
        }
        return out;
    }

    // A list of rows, each a list of `columns` numbers.
    std::vector<std::vector<double>> table(const std::string &key, size_t columns) {
        XmlRpc::XmlRpcValue              value;
        std::vector<std::vector<double>> out;
        if (!fetch(key, value)) {
            return out;
        }
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() == 0) {
            wrongType(key, "a list of rows");
            return out;
        }
        for (int i = 0; i < value.size(); ++i) {
            std::vector<double> row(columns, 0.0);
            if (!toNumbers(value[i], columns, row)) {
                wrongType(key, "rows of " + std::to_string(columns) + " numbers");
                out.clear();
                return out;
            }
            out.push_back(row);
        }
        return out;
    }

    // Records a problem with `key` when a value read from it breaks a rule, e.g. require(rate > 0, "rate_hz", "positive").
    void require(bool condition, const std::string &key, const std::string &rule) {
        if (!condition) {
            errors_.push_back(ns_ + "/" + key + " must be " + rule);
        }
    }

    // Every failed read, one per line; empty when all reads succeeded.
    std::string errors() const {
        std::string out;
        for (const std::string &line : errors_) {
            out += "  " + line + "\n";
        }
        return out;
    }

    // Keys under this namespace that nothing read, one per line: typos or leftovers.
    std::string unreadKeys() const {
        std::vector<std::string> names;
        ros::param::getParamNames(names);

        std::string out;
        for (const std::string &name : names) {
            if (name.compare(0, ns_.size() + 1, ns_ + "/") == 0 && read_.count(name) == 0) {
                out += "  " + name + " is set but nothing reads it\n";
            }
        }
        return out;
    }

private:
    bool fetch(const std::string &key, XmlRpc::XmlRpcValue &value) {
        const std::string name = ns_ + "/" + key;
        read_.insert(name);
        if (!ros::param::get(name, value)) {
            errors_.push_back(name + " is missing");
            return false;
        }
        return true;
    }

    void wrongType(const std::string &key, const std::string &expected) {
        errors_.push_back(ns_ + "/" + key + " must be " + expected);
    }

    static bool toNumber(XmlRpc::XmlRpcValue &value, double &out) {
        if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            out = static_cast<double>(value);
            return true;
        }
        if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            out = static_cast<int>(value);
            return true;
        }
        return false;
    }

    static bool toNumbers(XmlRpc::XmlRpcValue &value, size_t count, std::vector<double> &out) {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray
            || static_cast<size_t>(value.size()) != count) {
            return false;
        }
        for (int i = 0; i < value.size(); ++i) {
            if (!toNumber(value[i], out[i])) {
                return false;
            }
        }
        return true;
    }

    std::string              ns_;
    std::set<std::string>    read_;
    std::vector<std::string> errors_;
};

}  // namespace params

#endif  // PARAMS_PARAMS_H
