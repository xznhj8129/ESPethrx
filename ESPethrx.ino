#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#include <errno.h>
#include <esp_mac.h>   // <-- IDF v5 MAC API

// ===== RUNTIME =====
#define ETH_AP_GRACE_MS 10000 // wait this long after boot for ETH before starting AP

// ===== UART to FC 
#define UART_BAUD   115200
#define UART_RX_PIN 33      // FC TX -> GPIO33 "CFG"
#define UART_TX_PIN 32      // FC RX <- GPIO32 "485EN"

// ===== WT32-ETH01 Ethernet (LAN8720 RMII) =====
static const int ETH_ADDR      = 1;    // PHY address
static const int ETH_POWER_PIN = 16;   // enables 50 MHz osc on GPIO0
static const int ETH_MDC_PIN   = 23;
static const int ETH_MDIO_PIN  = 18;

// ===== AP fallback (when no Ethernet link) =====
#define AP_SSID     "FC-Configurator"
#define AP_PASSWORD "" // open
static const IPAddress AP_IP(192,168,0,1);
static const IPAddress AP_MASK_DEF(255,255,255,0);

// ===== Defaults =====
#define MSP_DEF_PORT     5760
#define MAVLINK_DEF_PORT 14550

// ===== Globals =====
HardwareSerial& FC = Serial2;
Preferences prefs;
WebServer server(80);

enum Mode : uint8_t { MODE_MSP = 0, MODE_MAVLINK = 1 };

struct Settings {
  bool     useDHCP;
  String   staticIP;     // empty -> DHCP
  String   gw;           // optional; default .1 of subnet
  String   mask;         // optional; default /24
  uint8_t  id;           // 0..255
  Mode     mode;         // MSP or MAVLink
  String   hostname;     // e.g. fc4
  uint16_t udpPort;      // 0 = mode default
} settings;

static bool ap_up = false;
static bool eth_ip_ready = false;

static int udp_fd = -1;
static uint16_t udp_active_port = 0;

static sockaddr_in last_peer{};
static bool peer_valid = false;
static uint32_t peer_last_ms = 0;
static const uint32_t PEER_TIMEOUT_MS = 60000; // forgiving for lazy clients

static uint32_t boot_ms = 0;

// ===== Helpers =====
static IPAddress defaultGWFrom(IPAddress ip){ return IPAddress(ip[0], ip[1], ip[2], 1); }
static bool ipIsSet(const IPAddress& ip){ return !(ip[0]==0 && ip[1]==0 && ip[2]==0 && ip[3]==0); }
static uint16_t modeDefaultPort(){ return (settings.mode==MODE_MAVLINK) ? MAVLINK_DEF_PORT : MSP_DEF_PORT; }
static uint16_t current_udp_port(){ return settings.udpPort ? settings.udpPort : modeDefaultPort(); }

static void plainOK(const char* msg="ok"){ server.send(200,"text/plain",msg); }
static void plainErr(const char* msg){ server.send(400,"text/plain",msg); }

static String urlDecode(const String& in){
  String out; out.reserve(in.length());
  for(size_t i=0;i<in.length();++i){
    char c=in[i];
    if(c=='%' && i+2<in.length()){
      auto hex=[](char x)->int{ if(x>='0'&&x<='9')return x-'0'; if(x>='A'&&x<='F')return x-'A'+10; if(x>='a'&&x<='f')return x-'a'+10; return -1; };
      int hi=hex(in[i+1]), lo=hex(in[i+2]);
      if(hi>=0&&lo>=0){ out+=char((hi<<4)|lo); i+=2; continue; }
    }
    out += (c=='+') ? ' ' : c;
  }
  return out;
}
static String stripQuotes(String s){
  s.trim();
  if(s.length()>=2 && s[0]=='"' && s[s.length()-1]=='"'){ s.remove(s.length()-1); s.remove(0,1); }
  return s;
}
static bool parseKVPath(const String& rawUri, String& key, String& val){
  String uri=urlDecode(rawUri);
  if(!uri.startsWith("/set/")) return false;
  int eq=uri.indexOf('=',5); if(eq<0) return false;
  key=uri.substring(5,eq); val=stripQuotes(uri.substring(eq+1)); return true;
}
static String macToStr(const uint8_t m[6]){
  char b[18];
  snprintf(b,sizeof(b),"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]);
  return String(b);
}
// robust ETH MAC getter (works even before ETH driver reports one)
static void getEthMac(uint8_t mac[6]){
  // Try ETH driver first
  ETH.macAddress(mac);
  bool allZero = true;
  for(int i=0;i<6;i++){ if(mac[i]){ allZero=false; break; } }
  if(allZero){
    // IDF v5 path
    esp_read_mac(mac, ESP_MAC_ETH);
  }
}

// ===== Persist =====
static void saveSettings(){
  prefs.begin("fcbridge", false);
  prefs.putBool("useDHCP", settings.useDHCP);
  prefs.putString("staticIP", settings.staticIP);
  prefs.putString("gw", settings.gw);
  prefs.putString("mask", settings.mask);
  prefs.putUChar("id", settings.id);
  prefs.putUChar("mode", (uint8_t)settings.mode);
  prefs.putString("host", settings.hostname);
  prefs.putUShort("port", settings.udpPort);
  prefs.end();
}
static void loadSettings(){
  prefs.begin("fcbridge", true);
  settings.useDHCP  = prefs.getBool("useDHCP", true);
  settings.staticIP = prefs.getString("staticIP", "");
  settings.gw       = prefs.getString("gw", "");
  settings.mask     = prefs.getString("mask", "");
  settings.id       = prefs.getUChar("id", (uint8_t)(ESP.getEfuseMac() & 0xFF));
  settings.mode     = (Mode)prefs.getUChar("mode", (uint8_t)MODE_MSP);
  settings.hostname = prefs.getString("host", String("fcbridge-") + String(settings.id));
  settings.udpPort  = prefs.getUShort("port", (uint16_t)0);
  prefs.end();
}

// ===== AP (fallback) =====
static void startAP(){
  if(ap_up) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK_DEF);
  WiFi.softAP(AP_SSID, (AP_PASSWORD[0]?AP_PASSWORD:nullptr));
  ap_up=true;

  server.on("/info", HTTP_GET, [](){
    uint8_t ethmac[6]; getEthMac(ethmac);
    String body = "mode=" + String(settings.mode==MODE_MSP?"msp":"mavlink") +
                  "\nid=" + String(settings.id) +
                  "\nhostname=" + settings.hostname +
                  "\nuse_dhcp=" + String(settings.useDHCP?"1":"0") +
                  "\nstatic_ip=" + settings.staticIP +
                  "\ngw=" + settings.gw +
                  "\nmask=" + settings.mask +
                  "\nport=" + String(current_udp_port()) +
                  "\neth_mac=" + macToStr(ethmac) + "\n";
    server.send(200,"text/plain",body);
  });

  server.on("/mac", HTTP_GET, [](){
    uint8_t ethmac[6]; getEthMac(ethmac);
    server.send(200,"text/plain",macToStr(ethmac));
  });

  server.on("/set", HTTP_GET, [](){
    bool touched=false, need_udp_refresh=false, ip_changed=false;

    if(server.hasArg("assign_ip")){
      String v=stripQuotes(urlDecode(server.arg("assign_ip")));
      IPAddress ip; if(!ip.fromString(v)) return plainErr("invalid ip");
      settings.staticIP=v; settings.useDHCP=false; touched=true; ip_changed=true;
    }
    if(server.hasArg("assign_gw")){
      String v=stripQuotes(urlDecode(server.arg("assign_gw")));
      IPAddress gw; if(!gw.fromString(v)) return plainErr("invalid gw");
      settings.gw=v; touched=true; ip_changed=true;
    }
    if(server.hasArg("assign_mask")){
      String v=stripQuotes(urlDecode(server.arg("assign_mask")));
      IPAddress m; if(!m.fromString(v)) return plainErr("invalid mask");
      settings.mask=v; touched=true; ip_changed=true;
    }
    if(server.hasArg("use_dhcp")){
      String v=stripQuotes(urlDecode(server.arg("use_dhcp")));
      settings.useDHCP=(v=="1"||v=="true"); touched=true; ip_changed=true;
    }
    if(server.hasArg("assign_id")){
      int n=stripQuotes(urlDecode(server.arg("assign_id"))).toInt();
      if(n<0||n>255) return plainErr("id 0..255");
      settings.id=(uint8_t)n; touched=true;
    }
    if(server.hasArg("hostname")){
      String h=stripQuotes(urlDecode(server.arg("hostname")));
      if(h.length()<1||h.length()>31) return plainErr("hostname 1..31");
      settings.hostname=h; touched=true;
    }
    if(server.hasArg("mode")){
      String m=stripQuotes(urlDecode(server.arg("mode"))); m.toLowerCase();
      if(m=="msp") settings.mode=MODE_MSP;
      else if(m=="mavlink") settings.mode=MODE_MAVLINK;
      else return plainErr("mode msp|mavlink");
      touched=true; need_udp_refresh=true;
    }
    if(server.hasArg("port")){
      int p=stripQuotes(urlDecode(server.arg("port"))).toInt();
      if(p<1||p>65535) return plainErr("port 1..65535");
      settings.udpPort=(uint16_t)p; touched=true; need_udp_refresh=true;
    }

    if(touched){ saveSettings(); }

    if(ip_changed) {
      server.send(200,"text/plain","ok (reboot to apply IP settings)");
    } else if(need_udp_refresh) {
      server.send(200,"text/plain","ok (port/mode applied)");
    } else if(touched) {
      plainOK();
    } else {
      plainErr("no args");
    }
  });

  server.on("/reboot", HTTP_GET, [](){
    server.send(200,"text/plain","rebooting");
    delay(150);
    ESP.restart();
  });

  server.onNotFound([](){
    String k,v;
    if(parseKVPath(server.uri(),k,v)){
      if(k=="assign_ip"){
        IPAddress ip; if(!ip.fromString(v)) return plainErr("invalid ip");
        settings.staticIP=v; settings.useDHCP=false; saveSettings();
        server.send(200,"text/plain","ok (reboot to apply IP settings)"); return;
      }
      if(k=="assign_gw"){
        IPAddress gw; if(!gw.fromString(v)) return plainErr("invalid gw");
        settings.gw=v; saveSettings(); server.send(200,"text/plain","ok (reboot to apply IP settings)"); return;
      }
      if(k=="assign_mask"){
        IPAddress m; if(!m.fromString(v)) return plainErr("invalid mask");
        settings.mask=v; saveSettings(); server.send(200,"text/plain","ok (reboot to apply IP settings)"); return;
      }
      if(k=="use_dhcp"){
        settings.useDHCP=(v=="1"||v=="true"); saveSettings();
        server.send(200,"text/plain","ok (reboot to apply IP settings)"); return;
      }
      if(k=="assign_id"){
        int n=v.toInt(); if(n<0||n>255) return plainErr("id 0..255");
        settings.id=(uint8_t)n; saveSettings(); return plainOK();
      }
      if(k=="hostname"){
        if(v.length()<1||v.length()>31) return plainErr("hostname 1..31");
        settings.hostname=v; saveSettings(); return plainOK();
      }
      if(k=="mode"){
        v.toLowerCase(); if(v=="msp") settings.mode=MODE_MSP; else if(v=="mavlink") settings.mode=MODE_MAVLINK; else return plainErr("mode msp|mavlink");
        saveSettings(); return plainOK();
      }
      if(k=="port"){
        int p=v.toInt(); if(p<1||p>65535) return plainErr("port 1..65535");
        settings.udpPort=(uint16_t)p; saveSettings(); return plainOK();
      }
    }
    server.send(404,"text/plain","not found");
  });

  server.begin();
  Serial.println("[AP] up at 192.168.0.1");
}
static void stopAP(){
  if(!ap_up) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  ap_up=false;
  Serial.println("[AP] stopped");
}

// ===== UDP bridge =====
static void close_udp(){
  if(udp_fd>=0){ ::close(udp_fd); udp_fd=-1; }
  udp_active_port=0; peer_valid=false;
  Serial.println("[UDP] closed");
}
static void start_udp(uint16_t port){
  if(udp_fd>=0 && udp_active_port==port) return;
  close_udp();
  udp_fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if(udp_fd<0){ Serial.printf("[UDP] socket() fail: %d\n", errno); return; }
  int on=1;
  ::setsockopt(udp_fd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
  ::setsockopt(udp_fd,SOL_SOCKET,SO_BROADCAST,&on,sizeof(on));
  sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(port); a.sin_addr.s_addr=htonl(INADDR_ANY);
  if(::bind(udp_fd,(sockaddr*)&a,sizeof(a))<0){ Serial.printf("[UDP] bind() fail: %d\n", errno); close_udp(); return; }
  fcntl(udp_fd, F_SETFL, fcntl(udp_fd,F_GETFL,0)|O_NONBLOCK);
  udp_active_port=port;
  Serial.printf("[UDP] listening on %u (%s)\n", port, (settings.mode==MODE_MSP?"MSP":"MAVLink"));
}
static IPAddress calcBroadcast(){
  IPAddress ip=ETH.localIP(), mask=ETH.subnetMask();
  uint32_t ip32=(uint32_t)ip, mask32=(uint32_t)mask;
  return IPAddress(ip32 | ~mask32);
}
static void pump_udp_to_uart(){
  if(udp_fd<0) return;
  uint8_t buf[600];
  for(;;){
    sockaddr_in ra{}; socklen_t rl=sizeof(ra);
    int n=::recvfrom(udp_fd, buf, sizeof(buf), 0, (sockaddr*)&ra, &rl);
    if(n>0){
      FC.write(buf,n);
      if(!peer_valid || ra.sin_addr.s_addr!=last_peer.sin_addr.s_addr || ra.sin_port!=last_peer.sin_port){
        last_peer=ra; peer_valid=true;
        uint32_t a = ntohl(ra.sin_addr.s_addr);
        Serial.printf("[UDP] peer %u.%u.%u.%u:%u\n",
                      (unsigned)((a>>24)&0xFF),(unsigned)((a>>16)&0xFF),
                      (unsigned)((a>>8)&0xFF),(unsigned)(a&0xFF),
                      ntohs(ra.sin_port));
      }
      peer_last_ms=millis();
      continue;
    }
    if(n<0 && errno!=EWOULDBLOCK && errno!=EAGAIN){
      // ignore transient UDP errors
    }
    break;
  }
}
static void pump_uart_to_udp(){
  if(udp_fd<0){ while(FC.available()) (void)FC.read(); return; }
  if(peer_valid && (millis()-peer_last_ms > PEER_TIMEOUT_MS)) peer_valid=false;

  uint8_t buf[600];
  int n=FC.available();
  if(n<=0) return;
  if(n>(int)sizeof(buf)) n=sizeof(buf);
  n=FC.readBytes((char*)buf, n);

  // Always unicast to last peer if we have one
  if(peer_valid){
    (void)::sendto(udp_fd, buf, n, 0, (sockaddr*)&last_peer, sizeof(last_peer));
  }

  // MAVLink: broadcast until a peer talks
  if(settings.mode==MODE_MAVLINK && !peer_valid){
    IPAddress b=calcBroadcast();
    sockaddr_in ba{}; ba.sin_family=AF_INET; ba.sin_port=htons(current_udp_port());
    ba.sin_addr.s_addr = htonl((uint32_t)b);
    ::sendto(udp_fd, buf, n, 0, (sockaddr*)&ba, sizeof(ba));
  }
}

// ===== Ethernet events =====
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onArduinoEvent(arduino_event_id_t event, arduino_event_info_t){
  switch(event){
    case ARDUINO_EVENT_ETH_START: {
      ETH.setHostname(settings.hostname.c_str());
      if(!settings.useDHCP && settings.staticIP.length()){
        IPAddress ip; if(ip.fromString(settings.staticIP)){
          IPAddress gw, mask;
          if(!settings.gw.length() || !gw.fromString(settings.gw))  gw = defaultGWFrom(ip);
          if(!settings.mask.length() || !mask.fromString(settings.mask)) mask = AP_MASK_DEF;
          bool ok = ETH.config(ip, gw, mask);
          Serial.printf("[ETH] static IP %s %s (gw %s mask %s)\n",
                        settings.staticIP.c_str(), ok?"applied":"failed",
                        settings.gw.length()?settings.gw.c_str():String(gw.toString()).c_str(),
                        settings.mask.length()?settings.mask.c_str():String(mask.toString()).c_str());
        }
      }
      break;
    }
    case ARDUINO_EVENT_ETH_CONNECTED:
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_ip_ready = true;
      start_udp(current_udp_port());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      eth_ip_ready = false;
      close_udp();
      break;
    default: break;
  }
}
#else
static void onWifiEvent(WiFiEvent_t event){
  switch(event){
    case SYSTEM_EVENT_ETH_START:
      ETH.setHostname(settings.hostname.c_str());
      if(!settings.useDHCP && settings.staticIP.length()){
        IPAddress ip; if(ip.fromString(settings.staticIP)){
          IPAddress gw, mask;
          if(!settings.gw.length() || !gw.fromString(settings.gw))  gw = defaultGWFrom(ip);
          if(!settings.mask.length() || !mask.fromString(settings.mask)) mask = AP_MASK_DEF;
          bool ok = ETH.config(ip, gw, mask);
          Serial.printf("[ETH] static IP %s %s (gw %s mask %s)\n",
                        settings.staticIP.c_str(), ok?"applied":"failed",
                        settings.gw.length()?settings.gw.c_str():String(gw.toString()).c_str(),
                        settings.mask.length()?settings.mask.c_str():String(mask.toString()).c_str());
        }
      }
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      eth_ip_ready = true;
      start_udp(current_udp_port());
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      eth_ip_ready = false;
      close_udp();
      break;
    default: break;
  }
}
#endif

static void startETH(){
  WiFi.mode(WIFI_OFF); // only AP as fallback
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  WiFi.onEvent(onArduinoEvent);
  if(!ETH.begin(ETH_PHY_LAN8720, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_POWER_PIN, ETH_CLOCK_GPIO0_IN)){
    Serial.println("[ETH] begin failed");
  }
#else
  WiFi.onEvent(onWifiEvent);
  if(!ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_PHY_LAN8720, ETH_CLOCK_GPIO0_IN)){
    Serial.println("[ETH] begin failed");
  }
#endif
}

// ===== Setup / Loop =====
void setup(){
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("WT32-ETH01 UDP UART Bridge (ETH primary, AP fallback)");

  loadSettings();

  FC.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("[UART] RX=%d TX=%d %d baud\n", UART_RX_PIN, UART_TX_PIN, UART_BAUD);

  // Show MAC for DHCP reservations
  uint8_t ethmac[6]; getEthMac(ethmac);
  Serial.printf("[ETH] MAC %s\n", macToStr(ethmac).c_str());

  boot_ms = millis();
  startETH();
}

void loop(){
  // Grace period before bringing up AP
  bool link = ETH.linkUp();
  if(!link){
    if(!ap_up && millis() - boot_ms > ETH_AP_GRACE_MS) startAP();
  } else {
    if(ap_up) stopAP();
  }

  if(ap_up){
    server.handleClient();
  } else if(eth_ip_ready){
    // Rebind if mode/port changed
    static uint16_t last_port = 0;
    uint16_t want = current_udp_port();
    if(want != last_port){ start_udp(want); last_port = want; }

    pump_udp_to_uart();
    pump_uart_to_udp();
  }

  delay(1); // yield
}
