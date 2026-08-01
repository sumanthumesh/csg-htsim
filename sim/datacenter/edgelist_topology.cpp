// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "edgelist_topology.h"
#include "compositequeue.h"
#include "queue.h"
#include "randomqueue.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>

// Local helpers.  Deliberately static: connection_matrix.cpp already exports a
// global tokenize() with different splitting behaviour (it splits on a single
// delimiter and keeps empty fields), and we want neither the clash nor the
// behaviour.
static void el_tokenize(const string& str, vector<string>& out) {
    out.clear();
    stringstream ss(str);
    string tok;
    while (ss >> tok) {
        if (!tok.empty() && tok[tok.size() - 1] == '\r') {
            tok.erase(tok.size() - 1);   // tolerate CRLF files
        }
        if (!tok.empty()) {
            out.push_back(tok);
        }
    }
}

static string el_lower(const string& s) {
    string out = s;
    for (string::iterator i = out.begin(); i != out.end(); i++) {
        *i = std::tolower(*i);
    }
    return out;
}

static bool el_is_comment(const vector<string>& tokens) {
    return tokens.empty() || tokens[0][0] == '#';
}

EdgelistTopology::EdgelistTopology(QueueLoggerFactory* logger_factory, EventList* ev,
                                   mem_b queuesize, queue_type qt, queue_type sender_qt) {
    _no_of_hosts = 0;
    _no_of_switches = 0;
    _no_of_nodes = 0;
    _no_of_links = 0;
    _max_paths = DEFAULT_MAX_PATHS;

    _default_speed = 0;
    _default_latency = 0;
    _default_queuesize = 0;
    _default_switch_latency = 0;

    _logger_factory = logger_factory;
    _eventlist = ev;
    _queuesize = queuesize;
    _qt = qt;
    _sender_qt = sender_qt;
}

EdgelistTopology::~EdgelistTopology() {
    // Only the link records are ours.  Queues, pipes and switches are
    // EventSources registered with the EventList, and htsim leaves those to
    // be reclaimed at process exit - deleting them here would double-free.
    for (map<pair<uint32_t, uint32_t>, DirectedLink*>::iterator i = _links.begin();
         i != _links.end(); i++) {
        delete i->second;
    }
}

EdgelistTopology* EdgelistTopology::load(const char* filename, QueueLoggerFactory* logger_factory,
                                         EventList& eventlist, mem_b queuesize,
                                         queue_type qt, queue_type sender_qt) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open edgelist topology file " << filename << endl;
        exit(1);
    }
    EdgelistTopology* top = load(file, logger_factory, eventlist, queuesize, qt, sender_qt);
    file.close();
    return top;
}

EdgelistTopology* EdgelistTopology::load(istream& file, QueueLoggerFactory* logger_factory,
                                         EventList& eventlist, mem_b queuesize,
                                         queue_type qt, queue_type sender_qt) {
    EdgelistTopology* top = new EdgelistTopology(logger_factory, &eventlist, queuesize, qt, sender_qt);
    if (!top->parse(file)) {
        exit(1);
    }
    top->init_network();
    return top;
}

//
// Parsing
//

bool EdgelistTopology::parse_header(istream& file, string& first_body_line, int& linecount) {
    string line;
    first_body_line = "";
    vector<string> tokens;

    while (std::getline(file, line)) {
        linecount++;
        el_tokenize(line, tokens);
        if (el_is_comment(tokens)) {
            continue;
        }
        string key = el_lower(tokens[0]);
        if (key == "switch" || key == "link" || key == "route") {
            // Header is over.  Hand the line back so parse() can process it.
            first_body_line = line;
            break;
        }
        if (tokens.size() < 2) {
            cerr << "Malformed header line at line " << linecount << ": " << line << endl;
            return false;
        }
        if (key == "hosts") {
            _no_of_hosts = stoi(tokens[1]);
        } else if (key == "switches") {
            _no_of_switches = stoi(tokens[1]);
        } else if (key == "links") {
            _no_of_links = stoi(tokens[1]);
        } else if (key == "default_speed_gbps") {
            _default_speed = speedFromGbps(stof(tokens[1]));
        } else if (key == "default_latency_ns") {
            _default_latency = timeFromNs(stof(tokens[1]));
        } else if (key == "default_queue_bytes") {
            _default_queuesize = (mem_b)stoll(tokens[1]);
        } else if (key == "default_switch_latency_ns") {
            _default_switch_latency = timeFromNs(stof(tokens[1]));
        } else {
            cerr << "Unknown header key \"" << tokens[0] << "\" at line " << linecount << endl;
            return false;
        }
    }

    if (_no_of_hosts == 0) {
        cerr << "Missing or zero \"Hosts\" in header" << endl;
        return false;
    }
    if (_no_of_links == 0) {
        cerr << "Missing or zero \"Links\" in header" << endl;
        return false;
    }
    if (_default_speed == 0) {
        cerr << "Missing \"Default_speed_Gbps\" in header" << endl;
        return false;
    }
    // A zero default latency is legal (links may each set their own), but a
    // zero fallback queue size is not - it would silently drop everything.
    if (_default_queuesize == 0) {
        _default_queuesize = _queuesize;
    }
    if (_default_queuesize == 0) {
        cerr << "No queue size available: set Default_queue_bytes in the header" << endl;
        return false;
    }
    return true;
}

bool EdgelistTopology::parse_endpoint(const string& token, uint32_t& node) const {
    if (token.size() < 2) {
        return false;
    }
    char kind = std::tolower(token[0]);
    if (kind != 'h' && kind != 's') {
        return false;
    }
    for (size_t i = 1; i < token.size(); i++) {
        if (!isdigit(token[i])) {
            return false;
        }
    }
    uint32_t id = (uint32_t)stoul(token.substr(1));
    if (kind == 'h') {
        if (id >= _no_of_hosts) {
            return false;
        }
        node = id;
    } else {
        if (id >= _no_of_switches) {
            return false;
        }
        node = _no_of_hosts + id;
    }
    return true;
}

bool EdgelistTopology::parse_switch(const vector<string>& tokens, int linecount) {
    if (tokens.size() < 2) {
        cerr << "Malformed switch line at line " << linecount << endl;
        return false;
    }
    uint32_t node;
    if (!parse_endpoint(tokens[1], node) || is_host(node)) {
        cerr << "Bad switch id \"" << tokens[1] << "\" at line " << linecount << endl;
        return false;
    }
    uint32_t sw = node - _no_of_hosts;

    for (size_t i = 2; i + 1 < tokens.size(); i += 2) {
        string key = el_lower(tokens[i]);
        if (key == "switch_latency_ns") {
            _switch_latencies[sw] = timeFromNs(stof(tokens[i + 1]));
        } else {
            cerr << "Unknown switch attribute \"" << tokens[i] << "\" at line " << linecount << endl;
            return false;
        }
    }
    return true;
}

bool EdgelistTopology::parse_link(const vector<string>& tokens, int linecount) {
    if (tokens.size() < 3) {
        cerr << "Malformed link line at line " << linecount << endl;
        return false;
    }
    uint32_t a, b;
    if (!parse_endpoint(tokens[1], a) || !parse_endpoint(tokens[2], b)) {
        cerr << "Bad link endpoint at line " << linecount << ": " << tokens[1] << " " << tokens[2] << endl;
        return false;
    }
    if (a == b) {
        cerr << "Self-loop at line " << linecount << endl;
        return false;
    }

    linkspeed_bps speed = _default_speed;
    simtime_picosec latency = _default_latency;
    mem_b queuesize = _default_queuesize;

    for (size_t i = 3; i + 1 < tokens.size(); i += 2) {
        string key = el_lower(tokens[i]);
        if (key == "speed_gbps") {
            speed = speedFromGbps(stof(tokens[i + 1]));
        } else if (key == "latency_ns") {
            latency = timeFromNs(stof(tokens[i + 1]));
        } else if (key == "queue_bytes") {
            queuesize = (mem_b)stoll(tokens[i + 1]);
        } else {
            cerr << "Unknown link attribute \"" << tokens[i] << "\" at line " << linecount << endl;
            return false;
        }
    }

    // One record per direction, so the two directions get independent queues.
    const uint32_t ends[2][2] = {{a, b}, {b, a}};
    for (int d = 0; d < 2; d++) {
        pair<uint32_t, uint32_t> key(ends[d][0], ends[d][1]);
        if (_links.count(key)) {
            cerr << "Duplicate link " << tokens[1] << " " << tokens[2]
                 << " at line " << linecount << endl;
            return false;
        }
        DirectedLink* link = new DirectedLink();
        link->src = ends[d][0];
        link->dst = ends[d][1];
        link->speed = speed;
        link->latency = latency;
        link->queuesize = queuesize;
        link->queue = NULL;
        link->pipe = NULL;
        _links[key] = link;
        _adjacency[ends[d][0]].push_back(ends[d][1]);
    }
    return true;
}

bool EdgelistTopology::parse_route(const vector<string>& tokens, int linecount) {
    if (tokens.size() < 3) {
        cerr << "Malformed route line at line " << linecount << endl;
        return false;
    }
    uint32_t src, dst;
    if (!parse_endpoint(tokens[1], src) || !parse_endpoint(tokens[2], dst)) {
        cerr << "Bad route endpoint at line " << linecount << endl;
        return false;
    }
    if (!is_host(src) || !is_host(dst)) {
        cerr << "Route endpoints must be hosts, at line " << linecount << endl;
        return false;
    }

    // Stored as a full node sequence so path search and pinned paths share one
    // representation downstream.
    vector<uint32_t> path;
    path.push_back(src);
    for (size_t i = 3; i < tokens.size(); i++) {
        uint32_t hop;
        if (!parse_endpoint(tokens[i], hop) || is_host(hop)) {
            cerr << "Route hop \"" << tokens[i] << "\" at line " << linecount
                 << " is not a switch" << endl;
            return false;
        }
        path.push_back(hop);
    }
    path.push_back(dst);

    _pinned_paths[pair<uint32_t, uint32_t>(src, dst)].push_back(path);
    return true;
}

bool EdgelistTopology::parse(istream& file) {
    string first_body_line;
    int linecount = 0;
    if (!parse_header(file, first_body_line, linecount)) {
        return false;
    }

    _no_of_nodes = _no_of_hosts + _no_of_switches;
    _adjacency.resize(_no_of_nodes);
    _switch_latencies.assign(_no_of_switches, _default_switch_latency);

    uint32_t links_seen = 0;
    string line = first_body_line;
    vector<string> tokens;

    // first_body_line is empty only if the file ended with the header.  It has
    // already been counted by parse_header, hence the guarded increment.
    bool have_line = !first_body_line.empty();
    while (have_line || std::getline(file, line)) {
        if (!have_line) {
            linecount++;
        }
        have_line = false;
        el_tokenize(line, tokens);
        if (el_is_comment(tokens)) {
            continue;
        }
        string cmd = el_lower(tokens[0]);
        if (cmd == "switch") {
            if (!parse_switch(tokens, linecount)) {
                return false;
            }
        } else if (cmd == "link") {
            if (!parse_link(tokens, linecount)) {
                return false;
            }
            links_seen++;
        } else if (cmd == "route") {
            if (!parse_route(tokens, linecount)) {
                return false;
            }
        } else {
            cerr << "Unknown directive \"" << tokens[0] << "\" at line " << linecount << endl;
            return false;
        }
    }

    if (links_seen != _no_of_links) {
        cerr << "Header declares " << _no_of_links << " links but " << links_seen
             << " were found" << endl;
        return false;
    }

    // Deterministic neighbour order, so path enumeration is reproducible
    // regardless of the order links appear in the file.
    for (uint32_t n = 0; n < _no_of_nodes; n++) {
        std::sort(_adjacency[n].begin(), _adjacency[n].end());
    }

    cout << "Edgelist topology: " << _no_of_hosts << " hosts, " << _no_of_switches
         << " switches, " << _no_of_links << " links" << endl;
    return true;
}

//
// Construction
//

BaseQueue* EdgelistTopology::alloc_queue(QueueLogger* logger, linkspeed_bps speed, mem_b queuesize) {
    switch (_qt) {
    case RANDOM:
        return new RandomQueue(speed, queuesize, *_eventlist, logger, memFromPkt(RANDOM_BUFFER));
    case COMPOSITE:
        return new CompositeQueue(speed, queuesize, *_eventlist, logger);
    default:
        cerr << "EdgelistTopology: unsupported switch queue type " << _qt
             << " - extend alloc_queue() to add it" << endl;
        abort();
    }
}

BaseQueue* EdgelistTopology::alloc_src_queue(QueueLogger* logger, linkspeed_bps speed) {
    switch (_sender_qt) {
    case FAIR_PRIO:
        return new FairPriorityQueue(speed, memFromPkt(FEEDER_BUFFER), *_eventlist, logger);
    default:
        cerr << "EdgelistTopology: unsupported sender queue type " << _sender_qt
             << " - extend alloc_src_queue() to add it" << endl;
        abort();
    }
}

void EdgelistTopology::init_network() {
    switches.resize(_no_of_switches);
    for (uint32_t s = 0; s < _no_of_switches; s++) {
        switches[s] = new Switch(*_eventlist, "Switch_" + ntoa(s));
    }

    for (map<pair<uint32_t, uint32_t>, DirectedLink*>::iterator i = _links.begin();
         i != _links.end(); i++) {
        DirectedLink* link = i->second;

        QueueLogger* logger = _logger_factory ? _logger_factory->createQueueLogger() : NULL;

        // The queue is the egress port of link->src.  Hosts get the sender
        // queue discipline, switches the switch-side one, matching the split
        // FatTreeTopology makes between alloc_src_queue and alloc_queue.
        string src_name = is_host(link->src) ? ("H" + ntoa(link->src))
                                             : ("S" + ntoa(link->src - _no_of_hosts));
        string dst_name = is_host(link->dst) ? ("H" + ntoa(link->dst))
                                             : ("S" + ntoa(link->dst - _no_of_hosts));

        if (is_host(link->src)) {
            link->queue = alloc_src_queue(logger, link->speed);
        } else {
            link->queue = alloc_queue(logger, link->speed, link->queuesize);
            // Switch latency is charged on egress from this switch.  It has to
            // ride on the pipe: source-routed packets never reach the Switch
            // object, so a latency held there would never be applied.
            link->latency += _switch_latencies[link->src - _no_of_hosts];
        }
        link->queue->setName(src_name + "->" + dst_name);

        link->pipe = new Pipe(link->latency, *_eventlist);
        link->pipe->setName("Pipe-" + src_name + "->" + dst_name);

        // Needed for PFC and for switch-level queue logging; harmless
        // otherwise, since forwarding is done by the source route.
        if (!is_host(link->dst)) {
            link->queue->setRemoteEndpoint(switches[link->dst - _no_of_hosts]);
        }
        if (!is_host(link->src)) {
            switches[link->src - _no_of_hosts]->addPort(link->queue);
        }
    }
}

//
// Path computation
//

EdgelistTopology::DirectedLink* EdgelistTopology::find_link(uint32_t src, uint32_t dst) const {
    map<pair<uint32_t, uint32_t>, DirectedLink*>::const_iterator i =
        _links.find(pair<uint32_t, uint32_t>(src, dst));
    return (i == _links.end()) ? NULL : i->second;
}

vector<vector<uint32_t>> EdgelistTopology::shortest_paths(uint32_t src, uint32_t dest) const {
    vector<vector<uint32_t>> paths;

    // Hop distance to dest, over links whose intermediate nodes are switches.
    // Hosts do not forward, so a path may only touch a host at its two ends -
    // otherwise we would happily route traffic "through" some other NPU.
    const uint32_t UNREACHED = (uint32_t)-1;
    vector<uint32_t> dist(_no_of_nodes, UNREACHED);
    std::queue<uint32_t> bfs;
    dist[dest] = 0;
    bfs.push(dest);
    while (!bfs.empty()) {
        uint32_t n = bfs.front();
        bfs.pop();
        // Expand out of dest itself, and out of switches; never out of another
        // host, which would make that host a transit node.
        if (n != dest && is_host(n)) {
            continue;
        }
        for (size_t k = 0; k < _adjacency[n].size(); k++) {
            uint32_t m = _adjacency[n][k];
            // _adjacency is symmetric (every link is stored in both
            // directions), so walking it backwards from dest is valid.
            if (dist[m] == UNREACHED) {
                dist[m] = dist[n] + 1;
                bfs.push(m);
            }
        }
    }

    if (dist[src] == UNREACHED) {
        return paths;
    }

    // Walk forward from src, only ever stepping to a neighbour that is one hop
    // closer to dest.  Every path enumerated this way is minimal, so the whole
    // set is equal-cost and uniform random selection over it is meaningful.
    vector<uint32_t> current;
    current.push_back(src);
    vector<size_t> branch;   // index into _adjacency[current.back()] to try next
    branch.push_back(0);

    while (!current.empty() && paths.size() < _max_paths) {
        uint32_t node = current.back();
        if (node == dest) {
            paths.push_back(current);
            current.pop_back();
            branch.pop_back();
            continue;
        }
        bool descended = false;
        while (branch.back() < _adjacency[node].size()) {
            uint32_t next = _adjacency[node][branch.back()];
            branch.back()++;
            if (dist[next] != dist[node] - 1) {
                continue;
            }
            if (next != dest && is_host(next)) {
                continue;   // no transit through hosts
            }
            current.push_back(next);
            branch.push_back(0);
            descended = true;
            break;
        }
        if (!descended) {
            current.pop_back();
            branch.pop_back();
        }
    }

    return paths;
}

Route* EdgelistTopology::build_route(const vector<uint32_t>& nodes) const {
    Route* route = new Route();
    for (size_t i = 0; i + 1 < nodes.size(); i++) {
        DirectedLink* link = find_link(nodes[i], nodes[i + 1]);
        if (!link) {
            cerr << "No link from node " << nodes[i] << " to " << nodes[i + 1]
                 << " while building route" << endl;
            delete route;
            return NULL;
        }
        route->push_back(link->queue);
        route->push_back(link->pipe);
    }
    return route;
}

vector<const Route*>* EdgelistTopology::get_bidir_paths(uint32_t src, uint32_t dest, bool reverse) {
    // reverse is ignored - see the note in the header.
    vector<const Route*>* paths = new vector<const Route*>();

    if (src >= _no_of_hosts || dest >= _no_of_hosts) {
        cerr << "get_bidir_paths called with non-host endpoint (" << src << ", " << dest << ")" << endl;
        exit(1);
    }
    if (src == dest) {
        return paths;
    }

    vector<vector<uint32_t>> node_paths;
    map<pair<uint32_t, uint32_t>, vector<vector<uint32_t>>>::const_iterator pinned =
        _pinned_paths.find(pair<uint32_t, uint32_t>(src, dest));
    if (pinned != _pinned_paths.end()) {
        node_paths = pinned->second;
    } else {
        node_paths = shortest_paths(src, dest);
    }

    if (node_paths.empty()) {
        cerr << "No path from host " << src << " to host " << dest
             << " - topology is disconnected for this pair" << endl;
        exit(1);
    }

    for (size_t i = 0; i < node_paths.size(); i++) {
        Route* route = build_route(node_paths[i]);
        if (!route) {
            exit(1);
        }
        route->set_path_id(i, node_paths.size());
        paths->push_back(route);
    }
    return paths;
}

vector<uint32_t>* EdgelistTopology::get_neighbours(uint32_t src) {
    if (src >= _no_of_nodes) {
        return NULL;
    }
    return new vector<uint32_t>(_adjacency[src]);
}

void EdgelistTopology::add_switch_loggers(Logfile& log, simtime_picosec sample_period) {
    for (uint32_t s = 0; s < _no_of_switches; s++) {
        switches[s]->add_logger(log, sample_period);
    }
}
