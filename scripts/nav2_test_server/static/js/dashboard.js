async function fetchHealth() {
    try {
      const res = await fetch("/api/monitor/health", { cache: "no-store" });
      if (!res.ok) throw new Error(res.status);
  
      const data = await res.json();
      document.getElementById("health").textContent =
        JSON.stringify(data, null, 2);
  
    } catch (e) {
      document.getElementById("health").textContent =
        "Health check failed: " + e;
    }
  }
  
  fetchHealth();
  setInterval(fetchHealth, 3000);
  