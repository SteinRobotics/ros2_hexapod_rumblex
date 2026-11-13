#include "sequence_loader.hpp"

#include <map>
#include <vector>

#include "yaml_fields.hpp"
namespace brain::sequence_detail {
namespace {
// Each sequence or snippet owns its variable scope; YAML anchors remain file-local.
class Variables {
   public:
    Variables(const YAML::Node& document, const std::filesystem::path& source) {
        if (const auto imports = document["imports"]) {
            if (!imports.IsSequence()) throw std::runtime_error("imports must be a list of file paths");
            if (source.empty()) throw std::runtime_error("Variable imports require a source file");
            for (const auto& entry : imports) {
                if (!entry.IsScalar() || entry.Scalar().empty())
                    throw std::runtime_error("Variable import requires a nonempty file path");
                const auto path = std::filesystem::canonical(source.parent_path() / entry.Scalar());
                const auto imported = YAML::LoadFile(path.string());
                keys(imported, {"variables"});
                add(imported["variables"], false);
            }
        }
        if (document["variables"]) add(document["variables"], true);
        // Validate even unused definitions so typos do not remain hidden.
        for (const auto& [name, value] : definitions_) variable(name, 0);
    }

    YAML::Node resolve(const YAML::Node& node, unsigned depth = 0) {
        if (depth >= 64) throw std::runtime_error("Variable/YAML nesting exceeds 64 levels");
        if (node.IsScalar()) {
            const auto text = node.Scalar();
            if (text.starts_with("${") && text.ends_with("}") && text.find('}', 2) == text.size() - 1)
                return variable(text.substr(2, text.size() - 3), depth + 1);
            return YAML::Clone(node);
        }
        YAML::Node result(node.Type());
        if (node.IsSequence()) {
            for (const auto& item : node) result.push_back(resolve(item, depth + 1));
        } else if (node.IsMap()) {
            for (const auto& entry : node)
                result.force_insert(YAML::Clone(entry.first), resolve(entry.second, depth + 1));
        }
        return result;
    }

   private:
    void add(const YAML::Node& values, bool local) {
        if (!values.IsMap()) throw std::runtime_error("Sequence variables must be a mapping");
        std::set<std::string> seen;
        for (const auto& entry : values) {
            const auto name = entry.first.as<std::string>();
            if (name.empty() || !seen.insert(name).second || (!local && definitions_.contains(name)))
                throw std::runtime_error("Empty, duplicate or conflicting variable: " + name);
            definitions_.insert_or_assign(name, YAML::Clone(entry.second));
        }
    }

    YAML::Node variable(const std::string& name, unsigned depth) {
        const auto found = definitions_.find(name);
        if (found == definitions_.end()) throw std::runtime_error("Undefined variable: " + name);
        if (const auto cached = resolved_.find(name); cached != resolved_.end())
            return YAML::Clone(cached->second);
        if (!active_.insert(name).second) throw std::runtime_error("Circular variable reference: " + name);
        const auto value = resolve(found->second, depth);
        active_.erase(name);
        resolved_.emplace(name, value);
        return YAML::Clone(value);
    }

    std::map<std::string, YAML::Node> definitions_;
    std::map<std::string, YAML::Node> resolved_;
    std::set<std::string> active_;
};

// Emit directly into one bounded catalog; includes and loops share the same budget.
void expandActions(const YAML::Node& actions, const std::filesystem::path& source,
                   std::vector<std::filesystem::path>& stack, YAML::Node& expanded, size_t& groups,
                   unsigned loopDepth = 0) {
    if (!actions.IsSequence() || actions.size() == 0)
        throw std::runtime_error("Expected nonempty actions sequence in " + source.string());
    for (const auto& action : actions) {
        if (!action.IsMap() || action.size() == 0)
            throw std::runtime_error("Expected nonempty action mapping");
        if (action["include"]) {
            keys(action, {"include"});
            if (source.empty()) throw std::runtime_error("Snippet includes require a source file");
            const auto path =
                std::filesystem::canonical(source.parent_path() / action["include"].as<std::string>());
            if (std::find(stack.begin(), stack.end(), path) != stack.end())
                throw std::runtime_error("Circular snippet include: " + path.string());
            if (stack.size() >= 64) throw std::runtime_error("Snippet nesting exceeds 64 files");
            stack.push_back(path);
            const auto snippet = YAML::LoadFile(path.string());
            keys(snippet, {"actions", "imports", "variables"});
            Variables variables(snippet, path);
            expandActions(variables.resolve(snippet["actions"]), path, stack, expanded, groups, loopDepth);
            stack.pop_back();
        } else if (action["loop"]) {
            keys(action, {"loop"});
            const auto loop = action["loop"];
            keys(loop, {"times", "actions"});
            const auto count = loop["times"].as<int>();
            if (count < 1 || count > 1000 || loopDepth >= 16)
                throw std::runtime_error("Loop requires 1..1000 repetitions and at most 16 nested loops");
            for (int i = 0; i < count; ++i)
                expandActions(loop["actions"], source, stack, expanded, groups, loopDepth + 1);
        } else {
            if (++groups > 10000) throw std::runtime_error("Catalog exceeds 10000 expanded action groups");
            expanded.push_back(action);
        }
    }
}
}  // namespace
YAML::Node loadSequences(const YAML::Node& document, const std::filesystem::path& source, size_t& groups) {
    YAML::Node sequences(YAML::NodeType::Sequence);
    if (document.IsMap() && document["sequences"]) {
        keys(document, {"sequences"});
        const auto list = document["sequences"];
        if (!list.IsSequence()) throw std::runtime_error("Expected sequences list");
        for (const auto& sequence : list) sequences.push_back(YAML::Clone(sequence));
    } else if (document.IsMap() && document["name"]) {
        sequences.push_back(YAML::Clone(document));
    } else {
        throw std::runtime_error("Expected a named sequence or sequences list");
    }
    for (auto sequence : sequences) {
        Variables variables(sequence, source);
        std::vector<std::filesystem::path> stack;
        if (!source.empty()) stack.push_back(source);
        YAML::Node expanded(YAML::NodeType::Sequence);
        expandActions(variables.resolve(sequence["actions"]), source, stack, expanded, groups);
        sequence["actions"] = expanded;
    }
    return sequences;
}
YAML::Node loadSequences(const std::filesystem::path& file, size_t& groups) {
    const auto path = std::filesystem::canonical(file);
    return loadSequences(YAML::LoadFile(path.string()), path, groups);
}
}  // namespace brain::sequence_detail
