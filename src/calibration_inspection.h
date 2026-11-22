// SPDX-License-Identifier: MIT
#pragma once
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <cJSON.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#endif

// Local-only, versioned inspection interface. Commands execute on the app thread.
// No arbitrary evaluation, file paths, device injection or acceptance overrides.
// Windows serves a named pipe at \\.\pipe\VDXRCalibration.<pid>; POSIX serves an
// AF_UNIX stream socket at $XDG_RUNTIME_DIR/VDXRCalibration.<pid>.sock (mode
// 0600, owner-only). The wire protocol is identical: one UTF-8 JSON request
// (32 KiB limit), one UTF-8 JSON response, then the client acknowledges receipt
// with {"ack":true}. One client is served at a time.
class CalibrationInspection {
    struct Request {
        std::string input,output;
        std::mutex mutex;std::condition_variable changed;
        bool done=false,expired=false;
    };
    std::atomic<bool> stopping{false};
    std::thread server;
    std::mutex mutex;
    std::queue<std::shared_ptr<Request>> queue;
    std::string state="{\"schema\":1,\"phase\":\"starting\"}";
#ifdef _WIN32
    std::wstring pipe;
#else
    std::string socketPath;
#endif
#ifdef _WIN32
    void serve() {
        HANDLE handle=CreateNamedPipeW(pipe.c_str(),PIPE_ACCESS_DUPLEX,
            PIPE_TYPE_MESSAGE|PIPE_READMODE_MESSAGE|PIPE_NOWAIT|PIPE_REJECT_REMOTE_CLIENTS,
            1,65536,65536,0,nullptr);
        if(handle==INVALID_HANDLE_VALUE)return;
        while(!stopping) {
            const BOOL connected=ConnectNamedPipe(handle,nullptr);
            if(!connected&&GetLastError()!=ERROR_PIPE_CONNECTED){Sleep(10);continue;}
            char buffer[32768];DWORD bytes=0;bool read=false;
            const auto start=std::chrono::steady_clock::now();
            while(!stopping&&std::chrono::steady_clock::now()-start<std::chrono::seconds(2)) {
                if(ReadFile(handle,buffer,sizeof(buffer)-1,&bytes,nullptr)&&bytes){read=true;break;}
                Sleep(5);
            }
            std::string response=respond(read ? std::string(buffer,bytes):std::string());
            DWORD written;WriteFile(handle,response.data(),DWORD(response.size()),&written,nullptr);
            // Client acknowledges reading the message before disconnect. This
            // preserves response delivery without an unbounded blocking flush.
            const auto acknowledgement=std::chrono::steady_clock::now();
            while(std::chrono::steady_clock::now()-acknowledgement<(stopping ? std::chrono::milliseconds(200):std::chrono::milliseconds(2000))) {
                DWORD received=0;if(ReadFile(handle,buffer,sizeof(buffer),&received,nullptr)&&received)break;Sleep(5);
            }
            DisconnectNamedPipe(handle);
        }
        CloseHandle(handle);
    }
#else
    // Shared response policy: 'state' is answered inline; anything else waits
    // (max 5 s) for the app thread to process it from the request queue.
    std::string respond(const std::string& input) {
        if(input.empty())return "{\"ok\":false,\"error\":\"request timeout\"}";
        cJSON* parsed=cJSON_Parse(input.c_str());
        const cJSON* operation=parsed ? cJSON_GetObjectItemCaseSensitive(parsed,"op"):nullptr;
        if(cJSON_IsString(operation)&&std::string(operation->valuestring)=="state") {
            std::lock_guard<std::mutex> lock(mutex);
            std::string response=state;cJSON_Delete(parsed);return response;
        }
        if(!parsed)return "{\"ok\":false,\"error\":\"invalid JSON\"}";
        auto request=std::make_shared<Request>();request->input=input;
        {std::lock_guard<std::mutex> lock(mutex);queue.push(request);}
        std::unique_lock<std::mutex> lock(request->mutex);
        request->changed.wait_for(lock,std::chrono::seconds(5),[&]{return request->done||stopping;});
        cJSON_Delete(parsed);
        if(request->done)return request->output;
        request->expired=true;return "{\"ok\":false,\"error\":\"app did not process command before expiry\"}";
    }
    static bool waitReadable(int fd,int milliseconds) {
        pollfd p{fd,POLLIN,0};
        return poll(&p,1,milliseconds)>0&&(p.revents&(POLLIN|POLLHUP));
    }
    void serve() {
        signal(SIGPIPE,SIG_IGN);
        const int socket=::socket(AF_UNIX,SOCK_STREAM,0);
        if(socket<0)return;
        sockaddr_un address{};
        address.sun_family=AF_UNIX;
        if(socketPath.size()>=sizeof(address.sun_path)) {close(socket);return;}
        std::memcpy(address.sun_path,socketPath.c_str(),socketPath.size()+1);
        ::unlink(socketPath.c_str()); // recycle a socket file from a crashed run
        if(bind(socket,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0||
           chmod(socketPath.c_str(),0600)!=0||listen(socket,1)!=0) {close(socket);return;}
        while(!stopping) {
            if(!waitReadable(socket,50))continue;
            const int client=accept(socket,nullptr,nullptr);
            if(client<0)continue;
            // Bound every blocking operation so a stuck client cannot wedge the
            // server thread (the pipe path is buffered instead on Windows).
            timeval timeout{2,0};
            setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
            setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
            // Read one request with the same 2 s window and 32 KiB limit.
            std::string input;char buffer[4096];
            const auto start=std::chrono::steady_clock::now();
            bool complete=false,overflow=false;
            while(!stopping&&std::chrono::steady_clock::now()-start<std::chrono::seconds(2)) {
                if(!waitReadable(client,20))continue;
                const ssize_t received=recv(client,buffer,sizeof(buffer),0);
                if(received<=0){complete=true;break;}
                if(input.size()+size_t(received)>32768){overflow=true;complete=true;break;}
                input.append(buffer,size_t(received));
                // One complete UTF-8 JSON document ends the request; trailing
                // whitespace is tolerated, anything else keeps reading.
                size_t end=input.size();
                while(end&&(input[end-1]==' '||input[end-1]=='\t'||input[end-1]=='\r'||input[end-1]=='\n'))--end;
                cJSON* probe=cJSON_Parse(input.substr(0,end).c_str());
                if(probe){cJSON_Delete(probe);complete=true;break;}
            }
            if(overflow)input.clear(); // oversized requests fail JSON validation like truncated pipe messages
            std::string response=respond(complete ? input:std::string());
            // Write the whole response; the client is expected to keep reading.
            size_t written=0;const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
            while(written<response.size()) {
                const ssize_t sent=send(client,response.data()+written,response.size()-written,MSG_NOSIGNAL);
                if(sent>0){written+=size_t(sent);continue;}
                if(sent<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)&&std::chrono::steady_clock::now()<deadline){waitReadable(client,20);continue;}
                break;
            }
            // Client acknowledges reading the message before disconnect. This
            // preserves response delivery without an unbounded blocking flush.
            const auto acknowledgement=std::chrono::steady_clock::now();
            while(std::chrono::steady_clock::now()-acknowledgement<(stopping ? std::chrono::milliseconds(200):std::chrono::milliseconds(2000))) {
                if(!waitReadable(client,20))continue;
                if(recv(client,buffer,sizeof(buffer),0)>0)break;
            }
            close(client);
        }
        close(socket);
        ::unlink(socketPath.c_str());
    }
#endif
public:
    CalibrationInspection() {
#ifdef _WIN32
        pipe=L"\\\\.\\pipe\\VDXRCalibration."+std::to_wstring(GetCurrentProcessId());
#else
        const char* runtime=getenv("XDG_RUNTIME_DIR");
        socketPath=std::string(runtime&&*runtime ? runtime:"/tmp")+"/VDXRCalibration."+std::to_string(getpid())+".sock";
#endif
        server=std::thread([this]{serve();});
    }
    ~CalibrationInspection(){stopping=true;if(server.joinable())server.join();
#ifndef _WIN32
        // The socket file is unlinked by the server thread; best effort here in
        // case the thread never started.
        if(!socketPath.empty())::unlink(socketPath.c_str());
#endif
    }
    // Endpoint path for clients; on Windows this is the UTF-8 pipe name.
    std::string endpoint() const {
#ifdef _WIN32
        std::string name;const std::wstring& wide=pipe;
        for(wchar_t c:wide)name.push_back(char(c));return name;
#else
        return socketPath;
#endif
    }
    void update(const std::string& snapshot){std::lock_guard<std::mutex> lock(mutex);state=snapshot;}
    void process(const std::function<std::string(const std::string&)>& handler) {
        for(int i=0;i<8;++i) {
            std::shared_ptr<Request> request;
            {std::lock_guard<std::mutex> lock(mutex);if(queue.empty())break;request=queue.front();queue.pop();}
            std::lock_guard<std::mutex> lock(request->mutex);
            if(request->expired)continue;
            try{request->output=handler(request->input);}
            catch(...){request->output="{\"ok\":false,\"error\":\"command failed\"}";}
            request->done=true;request->changed.notify_all();
        }
    }
};

inline std::string inspectionJson(cJSON* value) {
    char* text=cJSON_PrintUnformatted(value);std::string result=text ? text:"{}";cJSON_free(text);cJSON_Delete(value);return result;
}
