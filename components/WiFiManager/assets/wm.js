/**
 * WiFiManager JavaScript functionality
 */

// Global variables
let wmState = {
    selectedNetwork: null,
    scanInProgress: false,
    networks: []
};

// Utility functions
function $(id) {
    return document.getElementById(id);
}

function show(id) {
    $(id).style.display = 'block';
}

function hide(id) {
    $(id).style.display = 'none';
}

// Network scanning and display
function scanNetworks(callback) {
    if (wmState.scanInProgress) return;
    
    wmState.scanInProgress = true;
    
    fetch('/scan')
        .then(response => response.json())
        .then(networks => {
            wmState.networks = networks;
            wmState.scanInProgress = false;
            if (callback) callback(networks);
        })
        .catch(error => {
            console.error('Scan error:', error);
            wmState.scanInProgress = false;
            if (callback) callback([]);
        });
}

function displayNetworks(networks) {
    const container = $('networks');
    if (!container) return;
    
    if (networks.length === 0) {
        container.innerHTML = '<div style="text-align: center; padding: 20px;">No networks found<br><a href="javascript:refreshNetworks()">Refresh</a></div>';
        return;
    }
    
    container.innerHTML = '';
    networks.forEach(network => {
        const div = document.createElement('div');
        div.className = 'network';
        div.onclick = () => selectNetwork(network.ssid, div);
        
        const signalClass = getSignalClass(network.rssi);
        const security = network.encryption > 0 ? '🔒' : '';
        
        div.innerHTML = `
            <div>
                <div class="network-name">${escapeHtml(network.ssid)} ${security}</div>
                <div class="network-info">Channel ${network.channel}, ${network.rssi}dBm</div>
            </div>
            <div class="signal ${signalClass}">📶</div>
        `;
        
        container.appendChild(div);
    });
}

function selectNetwork(ssid, element) {
    if (wmState.selectedNetwork) {
        wmState.selectedNetwork.classList.remove('selected');
    }
    wmState.selectedNetwork = element;
    element.classList.add('selected');
    
    const ssidInput = $('s');
    if (ssidInput) {
        ssidInput.value = ssid;
    }
}

function refreshNetworks() {
    const container = $('networks');
    if (container) {
        container.innerHTML = '<div style="text-align: center; padding: 20px;">Scanning networks...</div>';
    }
    
    scanNetworks(displayNetworks);
}

// Signal strength helpers
function getSignalClass(rssi) {
    if (rssi > -30) return 'signal-5';
    if (rssi > -50) return 'signal-4';
    if (rssi > -60) return 'signal-3';
    if (rssi > -70) return 'signal-2';
    return 'signal-1';
}

function getSignalPercentage(rssi) {
    if (rssi <= -100) return 0;
    if (rssi >= -50) return 100;
    return 2 * (rssi + 100);
}

// Form validation
function validateForm() {
    const ssid = $('s');
    if (!ssid || !ssid.value.trim()) {
        alert('Please select or enter a network SSID');
        return false;
    }
    return true;
}

// Utility functions
function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function showMessage(message, type = 'info') {
    const msgDiv = document.createElement('div');
    msgDiv.className = `msg ${type}`;
    msgDiv.textContent = message;
    
    const container = document.querySelector('.wrap');
    if (container) {
        container.insertBefore(msgDiv, container.firstChild);
        
        // Auto-remove after 5 seconds
        setTimeout(() => {
            if (msgDiv.parentNode) {
                msgDiv.parentNode.removeChild(msgDiv);
            }
        }, 5000);
    }
}

// Page initialization
function initWiFiManager() {
    // Setup form validation
    const form = document.querySelector('form[action="/wifisave"]');
    if (form) {
        form.onsubmit = validateForm;
    }
    
    // Auto-refresh networks periodically
    if ($('networks')) {
        refreshNetworks();
        setInterval(refreshNetworks, 30000); // Refresh every 30 seconds
    }
    
    // Handle URL parameters for messages
    const urlParams = new URLSearchParams(window.location.search);
    const msg = urlParams.get('msg');
    const type = urlParams.get('type') || 'info';
    
    if (msg) {
        showMessage(decodeURIComponent(msg), type);
    }
}

// Initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initWiFiManager);
} else {
    initWiFiManager();
} 