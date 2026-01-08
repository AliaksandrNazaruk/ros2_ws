async function fetchJSON(url, elementId) {
    try {
      const res = await fetch(url, { cache: "no-store" });
      if (!res.ok) throw new Error(res.status);
  
      const data = await res.json();
      document.getElementById(elementId).textContent =
        JSON.stringify(data, null, 2);
  
    } catch (e) {
      document.getElementById(elementId).textContent =
        `Failed to fetch ${url}: ${e}`;
    }
  }
  
  function refreshAll() {
    fetchJSON("/api/monitor/nodes", "nodes");
    fetchJSON("/api/monitor/topics", "topics");
  }
  
  refreshAll();
  setInterval(refreshAll, 3000);
  